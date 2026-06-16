#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_adc/adc_continuous.h"
#include "esp_dsp.h"
#include "esp_private/esp_clk.h"
#include "driver/gpio.h"
#include "esp_timer.h"

/* Configuração do ADC: 1 canal, modo contínuo, resolução máxima */
#define MICEX_ADC_UNIT            ADC_UNIT_1
#define MICEX_ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN           ADC_ATTEN_DB_2_5
#define MICEX_ADC_BIT_WIDTH       SOC_ADC_DIGI_MAX_BITWIDTH

/* Tamanho de cada frame lido do ADC e do buffer interno do driver */
#define MICEX_ADC_FRAME_SIZE      512
#define MICEX_ADC_BUF_SIZE        (4 * MICEX_ADC_FRAME_SIZE)
#define MICEX_ADC_SAMPLE_FREQ     (20 * 1000)   /* fs = 20 kHz, acima do dobro da frequência mais alta a detetar */

/* Tamanho do bloco de amostras processado de cada vez */
#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048
#define MAX_FILT_IR_LEN                  200
#define CONV_OUT_LEN                     (MICEX_SOUND_SAMPLES_BUF_SIZE + 41 - 1)  /* tamanho da saída da convolução */

#define LED_GPIO_PIN                     11   /* LED usado para simular a porta */

/*
 * Frequências dos 3 tons, calculadas a partir da turma e dos NMEC do grupo:
 *   "0" = 500 Hz
 *   "1" = 1340 Hz
 *   "2" = 2180 Hz
 */
#define FREQ_0      500
#define FREQ_1      1340
#define FREQ_2      2180

/* Limiares usados para decidir se um tom está presente (ajustados por testes) */
#define MIN_SIGNAL_RMS           0.030f   /* energia mínima do sinal para considerar que há som */
#define DETECTION_THRESHOLD      0.008f   /* energia mínima à saída do filtro para considerar tom presente */
#define RATIO_THRESHOLD          0.15f    /* o quanto a energia do filtro deve dominar face ao sinal todo */

/* Estados possíveis da máquina de estados (sequência de abertura/fecho da porta) */
typedef enum {
    STATE_IDLE,
    STATE_OPEN_W_1,
    STATE_OPEN_W_2,
    STATE_OPEN_W_1_FINAL,
    STATE_CLOSE_W_1,
    STATE_CLOSE_W_0,
    STATE_CLOSE_W_1_FINAL,
    STATE_ERROR
} fsm_state_t;

#define SEQUENCE_TIMEOUT_US     (5 * 1000 * 1000)  /* tempo máximo entre tons antes de abortar a sequência */
#define DEBOUNCE_TIME_US        (150 * 1000)        /* tempo que um tom deve estar estável para ser aceite */

static adc_channel_t channel[1] = {ADC_CHANNEL_3};  /* canal do ADC ligado ao microfone */
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* Buffers alinhados a 16 bytes, exigido pelas funções do ESP-DSP */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};
__attribute__((aligned(16))) static float sound_samp_buf_ADC[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float sound_samp_buf_proc[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float conv_out_0[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_1[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_2[CONV_OUT_LEN];

#define PROCESSOR_TASK_STACK_SIZE       8192
#define PROCESSOR_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4 )

QueueHandle_t XQ;  /* fila usada para passar blocos de áudio entre tarefas */

/* Coeficientes dos 3 filtros FIR passa-banda, um por cada tom (500/1340/2180 Hz) */

/* Filtro do tom "0" (500 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_0[] = {
    -0.007247f, -0.007677f, -0.008860f, -0.010552f, -0.012375f, -0.013855f, -0.014478f, -0.013750f, -0.011256f, -0.006710f,
    -0.000000f,  0.008787f,  0.019362f,  0.031246f,  0.043803f,  0.056290f,  0.067917f,  0.077915f,  0.085603f,  0.090443f,
     0.092096f,  0.090443f,  0.085603f,  0.077915f,  0.067917f,  0.056290f,  0.043803f,  0.031246f,  0.019362f,  0.008787f,
    -0.000000f, -0.006710f, -0.011256f, -0.013750f, -0.014478f, -0.013855f, -0.012375f, -0.010552f, -0.008860f, -0.007677f,
    -0.007247f
};

/* Filtro do tom "1" (1340 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_1[] = {
    -0.003929f, -0.001133f,  0.002573f,  0.007696f,  0.013919f,  0.019814f,  0.023053f,  0.021117f,  0.012265f, -0.003541f,
    -0.024140f, -0.045340f, -0.061779f, -0.068318f, -0.061551f, -0.040999f, -0.009586f,  0.026811f,  0.060651f,  0.084558f,
     0.093178f,  0.084558f,  0.060651f,  0.026811f, -0.009586f, -0.040999f, -0.061551f, -0.068318f, -0.061779f, -0.045340f,
    -0.024140f, -0.003541f,  0.012265f,  0.021117f,  0.023053f,  0.019814f,  0.013919f,  0.007696f,  0.002573f, -0.001133f,
    -0.003929f
};

/* Filtro do tom "2" (2180 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_2[] = {
     0.003097f,  0.007037f,  0.009084f,  0.007166f, -0.000579f, -0.013004f, -0.024390f, -0.026355f, -0.013028f,  0.013558f,
     0.041967f,  0.055971f,  0.043618f,  0.005636f, -0.042425f, -0.076718f, -0.077529f, -0.040821f,  0.018040f,  0.071175f,
     0.092425f,  0.071175f,  0.018040f, -0.040821f, -0.077529f, -0.076718f, -0.042425f,  0.005636f,  0.043618f,  0.055971f,
     0.041967f,  0.013558f, -0.013028f, -0.026355f, -0.024390f, -0.013004f, -0.000579f,  0.007166f,  0.009084f,  0.007037f,
     0.003097f
};

float *filter_0 = filtro_real_tom_0;
float *filter_1 = filtro_real_tom_1;
float *filter_2 = filtro_real_tom_2;

int filter_len_0 = sizeof(filtro_real_tom_0) / sizeof(float);
int filter_len_1 = sizeof(filtro_real_tom_1) / sizeof(float);
int filter_len_2 = sizeof(filtro_real_tom_2) / sizeof(float);

/* Tira o nível DC ao sinal e ajusta a escala para valores entre -1 e 1 */
static void preprocess_buffer(float *buf, int n)
{
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += buf[i];
    mean /= (float)n;

    const float fixed_scale = 1.0f / 2048.0f;
    for (int i = 0; i < n; i++) {
        buf[i] = (buf[i] - mean) * fixed_scale;
    }
}

/* Calcula a energia (RMS) de um sinal */
static float calculate_rms(float *buf, int n)
{
    float acc = 0.0f;
    for (int i = 0; i < n; i++) acc += buf[i] * buf[i];
    return sqrtf(acc / n);
}

/* Passa o sinal por um filtro (convolução) e devolve a energia da saída, ignorando
   as bordas onde a convolução ainda não estabilizou */
static float detect_frequency_energy(float *input, int input_len, float *filter, int filter_len, float *conv_out)
{
    int out_len = input_len + filter_len - 1;
    memset(conv_out, 0, sizeof(float) * out_len);
    dsps_conv_f32(input, input_len, filter, filter_len, conv_out);

    int valid_start = filter_len - 1;
    int valid_len = out_len - 2 * valid_start;
    if (valid_len <= 0) {
        valid_start = 0;
        valid_len = out_len;
    }
    return calculate_rms(&conv_out[valid_start], valid_len);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
static void pv_processor_task(void *pvParam);

void app_main(void)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint32_t sb_count = 0;
    adc_continuous_evt_cbs_t cbs;
    adc_continuous_handle_t handle = NULL;

    /* Configura o pino do LED como saída, começando desligado */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO_PIN, 0);

    memset(result, 0x00, MICEX_ADC_FRAME_SIZE);

    s_task_handle = xTaskGetCurrentTaskHandle();

    /* Liga o callback que avisa quando o ADC tem dados novos prontos */
    cbs.on_conv_done = s_conv_done_cb;
    cbs.on_pool_ovf = NULL;

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    /* Fila com 1 espaço, para passar um bloco de áudio completo de cada vez */
    XQ = xQueueCreate(1, sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE);

    /* Arranca a tarefa que faz a deteção dos tons e a lógica da FSM */
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL);

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    /* Ciclo principal: espera o ADC avisar que há dados, lê-os e acumula amostras do microfone */
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                /* Percorre as amostras lidas e guarda só as do canal do microfone */
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = p->type2.channel;
                    uint32_t data = p->type2.data;

                    if (chan_num == channel[0]) {
                        sound_samp_buf_ADC[sb_count] = (float)data;
                        sb_count++;
                        /* Quando o buffer enche, prepara-o e envia-o para a tarefa de processamento */
                        if (sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) {
                            preprocess_buffer(sound_samp_buf_ADC, MICEX_SOUND_SAMPLES_BUF_SIZE);
                            xQueueSend(XQ, (void *)sound_samp_buf_ADC, 0);
                            sb_count = 0;
                        }
                    }
                }
                /* Sem delay aqui de propósito, para não perder amostras */
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }
    adc_continuous_stop(handle);
    adc_continuous_deinit(handle);
}

void pv_processor_task(void *pvParam)
{
    /* Valores máximos vistos até agora, só para efeitos de debug/log */
    static float peak_energy_0 = 0.0f;
    static float peak_energy_1 = 0.0f;
    static float peak_energy_2 = 0.0f;
    static float peak_rms = 0.0f;
    static int loop_counter = 0;

    /* Estado atual da máquina de estados e instante da última transição */
    static fsm_state_t current_state = STATE_IDLE;
    static int64_t last_state_change_time = 0;

    /* Controlo do pisca-pisca do LED quando há erro */
    static int64_t error_start_time = 0;
    static int64_t last_blink_time = 0;
    static uint32_t led_state = 0;

    /* Instante em que cada tom começou a ser detetado (0 = não está a ser detetado) */
    static int64_t tom_0_start_time = 0;
    static int64_t tom_1_start_time = 0;
    static int64_t tom_2_start_time = 0;
    static int last_registered_tom = -1;  /* último tom já aceite pela FSM, para não o aceitar 2x seguidas */

    for (;;) {

        /* Espera por um novo bloco de áudio pronto */
        xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY);

        /* Energia geral do bloco e energia à saída de cada filtro */
        float rms = calculate_rms(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE);

        float energy_0 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_0, filter_len_0, conv_out_0);
        float energy_1 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_1, filter_len_1, conv_out_1);
        float energy_2 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_2, filter_len_2, conv_out_2);

        if (rms > peak_rms) peak_rms = rms;
        if (energy_0 > peak_energy_0) peak_energy_0 = energy_0;
        if (energy_1 > peak_energy_1) peak_energy_1 = energy_1;
        if (energy_2 > peak_energy_2) peak_energy_2 = energy_2;

        int64_t current_time = esp_timer_get_time();

        /* Um tom conta como "detetado" se passar os 3 critérios: sinal suficiente,
           energia suficiente no filtro, e essa energia a dominar face ao sinal todo */
        bool tom_0_detectado = (rms > MIN_SIGNAL_RMS) && (energy_0 > DETECTION_THRESHOLD) && ((energy_0 / rms) > RATIO_THRESHOLD);
        bool tom_1_detectado = (rms > MIN_SIGNAL_RMS) && (energy_1 > DETECTION_THRESHOLD) && ((energy_1 / rms) > RATIO_THRESHOLD);
        bool tom_2_detectado = (rms > MIN_SIGNAL_RMS) && (energy_2 > DETECTION_THRESHOLD) && ((energy_2 / rms) > RATIO_THRESHOLD);

        /* Marca quando cada tom começou a ser detetado, para depois medir há quanto tempo está estável */
        if (tom_0_detectado) {
            if (tom_0_start_time == 0) tom_0_start_time = current_time;
        } else {
            tom_0_start_time = 0;
        }

        if (tom_1_detectado) {
            if (tom_1_start_time == 0) tom_1_start_time = current_time;
        } else {
            tom_1_start_time = 0;
        }

        if (tom_2_detectado) {
            if (tom_2_start_time == 0) tom_2_start_time = current_time;
        } else {
            tom_2_start_time = 0;
        }

        /* Um tom só é considerado "estável" depois de aguentar o tempo de debounce */
        bool tom_0_estavel = (tom_0_start_time > 0) && ((current_time - tom_0_start_time) >= DEBOUNCE_TIME_US);
        bool tom_1_estavel = (tom_1_start_time > 0) && ((current_time - tom_1_start_time) >= DEBOUNCE_TIME_US);
        bool tom_2_estavel = (tom_2_start_time > 0) && ((current_time - tom_2_start_time) >= DEBOUNCE_TIME_US);

        /* Liberta o "bloqueio" de um tom quando ele já não está a ser ouvido,
           para que possa voltar a ser aceite mais tarde */
        if (last_registered_tom == 0 && !tom_0_detectado) last_registered_tom = -1;
        if (last_registered_tom == 1 && !tom_1_detectado) last_registered_tom = -1;
        if (last_registered_tom == 2 && !tom_2_detectado) last_registered_tom = -1;

        /* Se demorar muito tempo entre tons, cancela a sequência a meio e volta a IDLE */
        if (current_state != STATE_IDLE && current_state != STATE_ERROR && (current_time - last_state_change_time) > SEQUENCE_TIMEOUT_US) {
            current_state = STATE_IDLE;
            last_registered_tom = -1;
            printf("\n>>> [FSM] Timeout excedido entre tons! Reset para IDLE.");
        }

        /* Máquina de estados: cada caso espera o próximo tom certo da sequência de
           abertura (0-1-2-1) ou de fecho (2-1-0-1); qualquer tom fora de ordem dá erro */
        switch (current_state) {

            case STATE_IDLE:
                if (tom_0_estavel && last_registered_tom != 0) {
                    current_state = STATE_OPEN_W_1;
                    last_state_change_time = current_time;
                    last_registered_tom = 0;
                    printf("\n>>> [FSM] Nota [0] aceite! Seq. de Abertura Iniciada... (Espera Tom 1)");
                } else if (tom_2_estavel && last_registered_tom != 2) {
                    current_state = STATE_CLOSE_W_1;
                    last_state_change_time = current_time;
                    last_registered_tom = 2;
                    printf("\n>>> [FSM] Nota [2] aceite! Seq. de Fecho Iniciada... (Espera Tom 1)");
                }
                break;

            case STATE_OPEN_W_1:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_OPEN_W_2;
                    last_state_change_time = current_time;
                    last_registered_tom = 1;
                    printf("\n>>> [FSM] Nota [1] aceite! (Espera Tom 2)");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_OPEN_W_2:
                if (tom_2_estavel && last_registered_tom != 2) {
                    current_state = STATE_OPEN_W_1_FINAL;
                    last_state_change_time = current_time;
                    last_registered_tom = 2;
                    printf("\n>>> [FSM] Nota [2] aceite! (Espera Tom 1 final)");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_1_estavel && last_registered_tom != 1)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_OPEN_W_1_FINAL:
                if (tom_1_estavel && last_registered_tom != 1) {
                    /* Último tom da sequência de abertura: liga o LED (porta aberta) */
                    current_state = STATE_IDLE;
                    last_registered_tom = 1;
                    gpio_set_level(LED_GPIO_PIN, 1);
                    printf("\n>>> [FSM] !!! ABERTURA COMPLETA (0121) !!! LED LIGADO");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_CLOSE_W_1:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_CLOSE_W_0;
                    last_state_change_time = current_time;
                    last_registered_tom = 1;
                    printf("\n>>> [FSM] Nota [1] aceite! (Espera Tom 0)");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_CLOSE_W_0:
                if (tom_0_estavel && last_registered_tom != 0) {
                    current_state = STATE_CLOSE_W_1_FINAL;
                    last_state_change_time = current_time;
                    last_registered_tom = 0;
                    printf("\n>>> [FSM] Nota [0] aceite! (Espera Tom 1 final)");
                } else if ((tom_1_estavel && last_registered_tom != 1) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_CLOSE_W_1_FINAL:
                if (tom_1_estavel && last_registered_tom != 1) {
                    /* Último tom da sequência de fecho: desliga o LED (porta fechada) */
                    current_state = STATE_IDLE;
                    last_registered_tom = 1;
                    gpio_set_level(LED_GPIO_PIN, 0);
                    printf("\n>>> [FSM] !!! FECHO COMPLETO (2101) !!! LED DESLIGADO");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_ERROR:
                /* Pisca o LED durante 5 segundos para sinalizar sequência errada */
                if ((current_time - error_start_time) < (5LL * 1000000LL)) {
                    if ((current_time - last_blink_time) >= 500000LL) {
                        led_state = !led_state;
                        gpio_set_level(LED_GPIO_PIN, led_state);
                        last_blink_time = current_time;
                    }
                } else {
                    /* Passados os 5s, desliga o LED e volta a aceitar sequências */
                    gpio_set_level(LED_GPIO_PIN, 0);
                    current_state = STATE_IDLE;
                    last_registered_tom = -1;
                    tom_0_start_time = 0;
                    tom_1_start_time = 0;
                    tom_2_start_time = 0;
                    printf("\n>>> [FSM] Tempo de erro esgotado. Reset para IDLE.");
                }
                break;

            /* Chegou aqui um tom fora da ordem esperada: entra em erro e pisca o LED */
            SEQUENCIA_ERRADA:
                current_state = STATE_ERROR;
                error_start_time = current_time;
                last_blink_time = current_time;
                led_state = 1;
                gpio_set_level(LED_GPIO_PIN, led_state);

                if (tom_0_estavel) last_registered_tom = 0;
                else if (tom_1_estavel) last_registered_tom = 1;
                else if (tom_2_estavel) last_registered_tom = 2;

                printf("\n>>> [FSM] ERRO NA SEQUÊNCIA! LED a piscar durante 5 segundos...");
                break;
        }

        /* A cada 12 blocos processados, imprime um resumo do estado atual para debug */
        loop_counter++;
        if (loop_counter >= 12) {
            const char *state_str = "DESCONHECIDO";
            switch(current_state) {
                case STATE_IDLE: state_str = "IDLE"; break;
                case STATE_ERROR: state_str = "ERRO (LED)"; break;
                case STATE_OPEN_W_1: state_str = "ABRIR: ESPERA 1"; break;
                case STATE_OPEN_W_2: state_str = "ABRIR: ESPERA 2"; break;
                case STATE_OPEN_W_1_FINAL: state_str = "ABRIR: ESPERA 1 F"; break;
                case STATE_CLOSE_W_1: state_str = "FECHAR: ESPERA 1"; break;
                case STATE_CLOSE_W_0: state_str = "FECHAR: ESPERA 0"; break;
                case STATE_CLOSE_W_1_FINAL: state_str = "FECHAR: ESPERA 1 F"; break;
            }

            char print_buf[512];
            snprintf(print_buf, sizeof(print_buf),
                     "\n=================================================\n"
                     "[FSM] Estado: %s | Bloqueio Nota: %d\n"
                     "[RMS] Atual: %.4f | Max: %.4f\n"
                     "[F0] Atual: %.4f | Estável: %d\n"
                     "[F1] Atual: %.4f | Estável: %d\n"
                     "[F2] Atual: %.4f | Estável: %d\n"
                     "=================================================\n",
                     state_str, last_registered_tom,
                     rms, peak_rms,
                     energy_0, tom_0_estavel,
                     energy_1, tom_1_estavel,
                     energy_2, tom_2_estavel);

            printf("%s", print_buf);
            loop_counter = 0;
        }
    }
}

/* Chamado pelo driver do ADC sempre que há um novo bloco de dados pronto;
   avisa a tarefa principal para ir buscar esses dados */
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

/* Configura o ADC em modo contínuo no canal do microfone */
static void continuous_adc_init(adc_channel_t *channel,
                                 uint8_t channel_num,
                                 adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = MICEX_ADC_BUF_SIZE,
        .conv_frame_size = MICEX_ADC_FRAME_SIZE,
    };

    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = MICEX_ADC_SAMPLE_FREQ,
        .conv_mode = MICEX_ADC_CONV_MODE,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};

    dig_cfg.pattern_num = channel_num;

    /* Define para cada canal a atenuação, número do canal, unidade ADC e resolução */
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = MICEX_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = MICEX_ADC_UNIT;
        adc_pattern[i].bit_width = MICEX_ADC_BIT_WIDTH;
    }

    dig_cfg.adc_pattern = adc_pattern;

    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}