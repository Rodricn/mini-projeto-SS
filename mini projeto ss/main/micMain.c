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

#define MICEX_ADC_UNIT            ADC_UNIT_1
#define MICEX_ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN           ADC_ATTEN_DB_2_5
#define MICEX_ADC_BIT_WIDTH       SOC_ADC_DIGI_MAX_BITWIDTH

#define MICEX_ADC_FRAME_SIZE      512
#define MICEX_ADC_BUF_SIZE        (4 * MICEX_ADC_FRAME_SIZE)
#define MICEX_ADC_SAMPLE_FREQ     (20 * 1000)   /* fs = 20 kHz: > 2x a maior frequência de tom (2180 Hz) com margem */

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048
#define MAX_FILT_IR_LEN                  200
#define CONV_OUT_LEN                     (MICEX_SOUND_SAMPLES_BUF_SIZE + 41 - 1)

#define LED_GPIO_PIN                     11

/*
 * Frequências dos tons (turma P1, NMECs 120009 / 119527):
 *   "0": 1 * 500              = 500  Hz
 *   "1": 500 + 700 + 7*20     = 1340 Hz   (soma_digitos = 12+25 = 37 -> mod10 = 7)
 *   "2": 1340 + 700 + 7*20    = 2180 Hz
 */
#define FREQ_0      500
#define FREQ_1      1340
#define FREQ_2      2180

/* Limiares de deteção, calibrados experimentalmente face ao ruído de fundo medido (RMS ~0.022 em silêncio) */
#define MIN_SIGNAL_RMS           0.030f
#define DETECTION_THRESHOLD      0.008f
#define RATIO_THRESHOLD          0.15f

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

#define SEQUENCE_TIMEOUT_US     (5 * 1000 * 1000)
#define DEBOUNCE_TIME_US        (150 * 1000)

static adc_channel_t channel[1] = {ADC_CHANNEL_3};
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* Alocação estática com ALIGN16, obrigatória para evitar erros no ESP-DSP */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};
__attribute__((aligned(16))) static float sound_samp_buf_ADC[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float sound_samp_buf_proc[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float conv_out_0[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_1[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_2[CONV_OUT_LEN];

#define PROCESSOR_TASK_STACK_SIZE       8192
#define PROCESSOR_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4 )

QueueHandle_t XQ;

/*
 * Filtros FIR passa-banda (Hamming, fs = 20 kHz, ordem 40 / 41 coeficientes).
 * Largura de banda de 100 Hz centrada em cada frequência de tom, garantindo
 * atenuação >= 25 dB nas frequências dos restantes tons (ver relatório, secção
 * de dimensionamento de filtros).
 */

/* Filtro passa-banda centrado em FREQ_0 = 500 Hz */
__attribute__((aligned(16))) float filtro_real_tom_0[] = {
    -0.007247f, -0.007677f, -0.008860f, -0.010552f, -0.012375f, -0.013855f, -0.014478f, -0.013750f, -0.011256f, -0.006710f,
    -0.000000f,  0.008787f,  0.019362f,  0.031246f,  0.043803f,  0.056290f,  0.067917f,  0.077915f,  0.085603f,  0.090443f,
     0.092096f,  0.090443f,  0.085603f,  0.077915f,  0.067917f,  0.056290f,  0.043803f,  0.031246f,  0.019362f,  0.008787f,
    -0.000000f, -0.006710f, -0.011256f, -0.013750f, -0.014478f, -0.013855f, -0.012375f, -0.010552f, -0.008860f, -0.007677f,
    -0.007247f
};

/* Filtro passa-banda centrado em FREQ_1 = 1340 Hz */
__attribute__((aligned(16))) float filtro_real_tom_1[] = {
    -0.003929f, -0.001133f,  0.002573f,  0.007696f,  0.013919f,  0.019814f,  0.023053f,  0.021117f,  0.012265f, -0.003541f,
    -0.024140f, -0.045340f, -0.061779f, -0.068318f, -0.061551f, -0.040999f, -0.009586f,  0.026811f,  0.060651f,  0.084558f,
     0.093178f,  0.084558f,  0.060651f,  0.026811f, -0.009586f, -0.040999f, -0.061551f, -0.068318f, -0.061779f, -0.045340f,
    -0.024140f, -0.003541f,  0.012265f,  0.021117f,  0.023053f,  0.019814f,  0.013919f,  0.007696f,  0.002573f, -0.001133f,
    -0.003929f
};

/* Filtro passa-banda centrado em FREQ_2 = 2180 Hz */
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

/* Remove o nível DC do buffer e normaliza pela amplitude máxima do ADC (12 bits -> 2048) */
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

static float calculate_rms(float *buf, int n)
{
    float acc = 0.0f;
    for (int i = 0; i < n; i++) acc += buf[i] * buf[i];
    return sqrtf(acc / n);
}

/*
 * Convolui 'input' com 'filter' e devolve o RMS da região "válida" da saída
 * (descartando as zonas de transiente nas extremidades, de tamanho filter_len-1
 * em cada lado), evitando que os efeitos de borda da convolução distorçam a
 * energia estimada.
 */
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

    cbs.on_conv_done = s_conv_done_cb;
    cbs.on_pool_ovf = NULL;

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    XQ = xQueueCreate(1, sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE);

    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL);

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
                    uint32_t chan_num = p->type2.channel;
                    uint32_t data = p->type2.data;

                    if (chan_num == channel[0]) {
                        sound_samp_buf_ADC[sb_count] = (float)data;
                        sb_count++;
                        if (sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) {
                            preprocess_buffer(sound_samp_buf_ADC, MICEX_SOUND_SAMPLES_BUF_SIZE);
                            xQueueSend(XQ, (void *)sound_samp_buf_ADC, 0);
                            sb_count = 0;
                        }
                    }
                }
                /* Sem vTaskDelay aqui: necessário para não perder frames a 20 kHz */
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
    static float peak_energy_0 = 0.0f;
    static float peak_energy_1 = 0.0f;
    static float peak_energy_2 = 0.0f;
    static float peak_rms = 0.0f;
    static int loop_counter = 0;

    static fsm_state_t current_state = STATE_IDLE;
    static int64_t last_state_change_time = 0;

    static int64_t error_start_time = 0;
    static int64_t last_blink_time = 0;
    static uint32_t led_state = 0;

    static int64_t tom_0_start_time = 0;
    static int64_t tom_1_start_time = 0;
    static int64_t tom_2_start_time = 0;
    static int last_registered_tom = -1;

    for (;;) {

        xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY);

        float rms = calculate_rms(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE);

        float energy_0 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_0, filter_len_0, conv_out_0);
        float energy_1 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_1, filter_len_1, conv_out_1);
        float energy_2 = detect_frequency_energy(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filter_2, filter_len_2, conv_out_2);

        if (rms > peak_rms) peak_rms = rms;
        if (energy_0 > peak_energy_0) peak_energy_0 = energy_0;
        if (energy_1 > peak_energy_1) peak_energy_1 = energy_1;
        if (energy_2 > peak_energy_2) peak_energy_2 = energy_2;

        int64_t current_time = esp_timer_get_time();

        bool tom_0_detectado = (rms > MIN_SIGNAL_RMS) && (energy_0 > DETECTION_THRESHOLD) && ((energy_0 / rms) > RATIO_THRESHOLD);
        bool tom_1_detectado = (rms > MIN_SIGNAL_RMS) && (energy_1 > DETECTION_THRESHOLD) && ((energy_1 / rms) > RATIO_THRESHOLD);
        bool tom_2_detectado = (rms > MIN_SIGNAL_RMS) && (energy_2 > DETECTION_THRESHOLD) && ((energy_2 / rms) > RATIO_THRESHOLD);

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

        bool tom_0_estavel = (tom_0_start_time > 0) && ((current_time - tom_0_start_time) >= DEBOUNCE_TIME_US);
        bool tom_1_estavel = (tom_1_start_time > 0) && ((current_time - tom_1_start_time) >= DEBOUNCE_TIME_US);
        bool tom_2_estavel = (tom_2_start_time > 0) && ((current_time - tom_2_start_time) >= DEBOUNCE_TIME_US);

        if (last_registered_tom == 0 && !tom_0_detectado) last_registered_tom = -1;
        if (last_registered_tom == 1 && !tom_1_detectado) last_registered_tom = -1;
        if (last_registered_tom == 2 && !tom_2_detectado) last_registered_tom = -1;

        if (current_state != STATE_IDLE && current_state != STATE_ERROR && (current_time - last_state_change_time) > SEQUENCE_TIMEOUT_US) {
            current_state = STATE_IDLE;
            last_registered_tom = -1;
            printf("\n>>> [FSM] Timeout excedido entre tons! Reset para IDLE.");
        }

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
                    current_state = STATE_IDLE;
                    last_registered_tom = 1;
                    gpio_set_level(LED_GPIO_PIN, 0);
                    printf("\n>>> [FSM] !!! FECHO COMPLETO (2101) !!! LED DESLIGADO");
                } else if ((tom_0_estavel && last_registered_tom != 0) || (tom_2_estavel && last_registered_tom != 2)) {
                    goto SEQUENCIA_ERRADA;
                }
                break;

            case STATE_ERROR:
                if ((current_time - error_start_time) < (5LL * 1000000LL)) {
                    if ((current_time - last_blink_time) >= 500000LL) {
                        led_state = !led_state;
                        gpio_set_level(LED_GPIO_PIN, led_state);
                        last_blink_time = current_time;
                    }
                } else {
                    gpio_set_level(LED_GPIO_PIN, 0);
                    current_state = STATE_IDLE;
                    last_registered_tom = -1;
                    /* Reinicia os temporizadores de estabilidade para evitar transições
                       espúrias caso um tom tenha ficado "preso" durante o erro */
                    tom_0_start_time = 0;
                    tom_1_start_time = 0;
                    tom_2_start_time = 0;
                    printf("\n>>> [FSM] Tempo de erro esgotado. Reset para IDLE.");
                }
                break;

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

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

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
