/* ********************************************************************************************************************************* Teste 
* Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonecsa, Luis Moutinho 2026/Apr.
 * * Tested:
 * ESP32-C6 DevKitC-1
 * * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, to allow higher frequencies
 * - Signal is processed by a Band-Pass filter, in the time-domain, to identify defined frequencies 
 * * Microphone is a MEMS Adafruit Silicon MEMS Microphone Breakout - SPW2430.
 * Supplied with 3.3-5V, output at DC pin has a 0.7 V and a 100 mVpp "when talking near". 
 * In my case I had around 1 V. So the attenuation cannot be 0 dB. 
 * I have used 2.5 dB (vref/0.7), to get 1.3 to 1.5 volts for Vref+ and avoid saturation
 * Check other mics to see if this is normal.  
 * * * Bibliography: 
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html
 * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_continuous.html 
 * https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-library.html      
 * * Based on the sample code  provided by EspressIF:
 * https://github.com/espressif/esp-idf/tree/47faecc3/examples/peripherals/adc/continuous_read 
 * * NOTE: must run idf.py add-dependency "espressif/esp-dsp" when creating a new project using dsp functionality
 ***********************************************************************************************************************************/ 

/* ********************************* * Includes
 ***********************************/
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
#include "driver/gpio.h" // Adicionado para controlo do LED periférico requisitado
#include "esp_timer.h"   // Adicionado para controlo de timeouts da FSM em microsegundos

/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define MICEX_ADC_FRAME_SIZE              512
#define MICEX_ADC_BUF_SIZE                (4 * MICEX_ADC_FRAME_SIZE)
#define MICEX_ADC_SAMPLE_FREQ             (20 * 1000)

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048
#define MAX_FILT_IR_LEN                  200

#define LED_GPIO_PIN                     11 // Requisito do enunciado: LED ligado ao GPIO11

/* *****************************************************************
 * FREQUÊNCIAS DOS TONS DEFINITIVAS
 * *****************************************************************/
#define FREQ_0      500  
#define FREQ_1      1340 
#define FREQ_2      2180 

/* *****************************************************************
 * LIMIARES DE DETECÇÃO (PONTO 1)
 * *****************************************************************/
#define DETECTION_THRESHOLD      35.0f  // Valor afinável baseado nos picos lidos no monitor
#define MIN_SIGNAL_RMS           0.05f  // Limiar mínimo absoluto para ignorar silêncio
#define RATIO_THRESHOLD          1.5f   // Rácio mínimo entre Energia Filtrada / RMS Global

/* *****************************************************************
 * DEFINIÇÕES DA MÁQUINA DE ESTADOS - FSM (Duas Sequências de 4 Tons)
 * *****************************************************************/
typedef enum {
    STATE_IDLE,                 // À espera do início de uma sequência (Tom 0 ou Tom 2)
    
    // Estados da Sequência de Abertura (0 -> 1 -> 2 -> 1)
    STATE_OPEN_W_1,             // Tom 0 detetado, à espera do Tom 1
    STATE_OPEN_W_2,             // Tom 1 detetado, à espera do Tom 2
    STATE_OPEN_W_1_FINAL,       // Tom 2 detetado, à espera do último Tom 1
    
    // Estados da Sequência de Fecho (2 -> 1 -> 0 -> 1)
    STATE_CLOSE_W_1,            // Tom 2 detetado, à espera do Tom 1
    STATE_CLOSE_W_0,            // Tom 1 detetado, à espera do Tom 0
    STATE_CLOSE_W_1_FINAL       // Tom 0 detetado, à espera do último Tom 1
} fsm_state_t;

#define SEQUENCE_TIMEOUT_US     (5 * 1000 * 1000) // Timeout de 5 segundos entre tons (em microsegundos)
#define DEBOUNCE_TIME_US        (150 * 1000)      // O tom deve soar por 150ms para ser considerado válido

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};

__attribute__((aligned(16))) 
adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES];

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192
#define PROCESSOR_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4 )

QueueHandle_t XQ;

/* *****************************************************************
 * FILTROS FIR FINAIS CALCULADOS (Ordem 40, Janela Hamming, Fs=20kHz)
 * *****************************************************************/

/* Filtro Passa-Banda de 41 Coeficientes para o Tom "0" (500 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_0[] = {
    -0.0034f, -0.0037f, -0.0037f, -0.0030f, -0.0014f,  0.0011f,  0.0044f,  0.0083f,  0.0125f,  0.0166f,
     0.0203f,  0.0232f,  0.0251f,  0.0259f,  0.0255f,  0.0238f,  0.0210f,  0.0173f,  0.0130f,  0.0082f,
     0.0034f,  0.0082f,  0.0130f,  0.0173f,  0.0210f,  0.0238f,  0.0255f,  0.0259f,  0.0251f,  0.0232f,
     0.0203f,  0.0166f,  0.0125f,  0.0083f,  0.0044f,  0.0011f, -0.0014f, -0.0030f, -0.0037f, -0.0037f,
    -0.0034f
};

/* Filtro Passa-Banda de 41 Coeficientes para o Tom "1" (1340 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_1[] = {
    -0.0016f,  0.0035f,  0.0046f, -0.0006f, -0.0054f, -0.0029f,  0.0049f,  0.0081f,  0.0013f, -0.0093f,
    -0.0105f,  0.0001f,  0.0133f,  0.0138f, -0.0015f, -0.0186f, -0.0201f,  0.0003f,  0.0264f,  0.0355f,
     0.0217f,  0.0355f,  0.0264f,  0.0003f, -0.0201f, -0.0186f, -0.0015f,  0.0138f,  0.0133f,  0.0001f,
    -0.0105f, -0.0093f,  0.0013f,  0.0081f,  0.0049f, -0.0029f, -0.0054f, -0.0006f,  0.0046f,  0.0035f,
    -0.0016f
};

/* Filtro Passa-Banda de 41 Coeficientes para o Tom "2" (2180 Hz) */
__attribute__((aligned(16))) float filtro_real_tom_2[] = {
     0.0033f,  0.0005f, -0.0049f, -0.0040f,  0.0030f,  0.0061f, -0.0010f, -0.0082f, -0.0019f,  0.0093f,
     0.0053f, -0.0098f, -0.0100f,  0.0085f,  0.0159f, -0.0051f, -0.0226f, -0.0011f,  0.0289f,  0.0125f,
    -0.0336f,  0.0125f,  0.0289f, -0.0011f, -0.0226f, -0.0051f,  0.0159f,  0.0085f, -0.0100f, -0.0098f,
     0.0053f,  0.0093f, -0.0019f, -0.0082f, -0.0010f,  0.0061f,  0.0030f, -0.0040f, -0.0049f,  0.0005f,
     0.0033f
};

/* *****************************************************************
 * ARRAYS DOS FILTROS (Mapeados para os teus filtros finais)
 * *****************************************************************/
float *filter_0 = filtro_real_tom_0;
float *filter_1 = filtro_real_tom_1;
float *filter_2 = filtro_real_tom_2;

int filter_len_0 = sizeof(filtro_real_tom_0) / sizeof(float);
int filter_len_1 = sizeof(filtro_real_tom_1) / sizeof(float);
int filter_len_2 = sizeof(filtro_real_tom_2) / sizeof(float);

/* *************************************************************** * PREPROCESSAMENTO DO SINAL (PONTO 1)
 * *****************************************************************/
static void preprocess_buffer(float *buf, int n)
{
    float mean = 0.0f;

    /* cálculo do offset DC */
    for (int i = 0; i < n; i++) {
        mean += buf[i];
    }

    mean /= (float)n;

    float max_abs = 0.0f;

    /* remoção do DC */
    for (int i = 0; i < n; i++) {

        buf[i] = buf[i] - mean;

        float a = fabsf(buf[i]);

        if (a > max_abs)
            max_abs = a;
    }

    /* normalização */
    if (max_abs > 0.0f) {

        float scale = 1.0f / max_abs;

        for (int i = 0; i < n; i++) {
            buf[i] *= scale;
        }
    }
}

/* *****************************************************************
 * PONTO 2 — ENERGIA RMS
 * *****************************************************************/
static float calculate_rms(float *buf, int n)
{
    float acc = 0.0f;

    for (int i = 0; i < n; i++) {
        acc += buf[i] * buf[i];
    }

    return sqrtf(acc / n);
}

/* *****************************************************************
 * PONTO 3 — FILTRAGEM FIR + DETECÇÃO
 * *****************************************************************/
static float detect_frequency_energy(float *input,
                                     int input_len,
                                     float *filter,
                                     int filter_len)
{
    int out_len = input_len + filter_len - 1;

    float *conv_out =
        heap_caps_malloc(sizeof(float) * out_len, MALLOC_CAP_DMA);

    if (conv_out == NULL) {
        ESP_LOGE(TAG, "Erro a alocar memória para convolução");
        return 0.0f;
    }

    /* limpa output */
    memset(conv_out, 0, sizeof(float) * out_len);

    /* convolução FIR por Hardware */
    dsps_conv_f32(input,
                  input_len,
                  filter,
                  filter_len,
                  conv_out);

    /* energia RMS da saída filtrada */
    float rms = calculate_rms(conv_out, out_len);

    free(conv_out);

    return rms;
}

/* *************************************************************** * Function prototypes 
 * *****************************************************************/
static void continuous_adc_init(adc_channel_t *channel,
                                uint8_t channel_num,
                                adc_continuous_handle_t *out_handle);

static bool s_conv_done_cb(adc_continuous_handle_t handle,
                           const adc_continuous_evt_data_t *edata,
                           void *user_data);

static void pv_processor_task(void *pvParam);

/******************************************************************* * The main task 
 * *******************************************************************/
void app_main(void)
{
    esp_err_t ret;
    esp_err_t parse_ret;

    uint32_t ret_num = 0;
    uint32_t sb_count = 0;
    uint32_t num_parsed_samples = 0;

    adc_continuous_evt_cbs_t cbs;
    adc_continuous_handle_t handle = NULL;

    float *sound_samp_buf_ADC;

    /* Configuração do GPIO11 para o LED da porta */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_GPIO_PIN, 0); // Porta fechada/desligada por defeito

    memset(result, 0x00, MICEX_ADC_FRAME_SIZE);

    sound_samp_buf_ADC =
        heap_caps_malloc(sizeof(float) *
                         MICEX_SOUND_SAMPLES_BUF_SIZE,
                         MALLOC_CAP_DMA);

    s_task_handle = xTaskGetCurrentTaskHandle();

    cbs.on_conv_done = s_conv_done_cb;
    cbs.on_pool_ovf = NULL;

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    XQ = xQueueCreate(1,
                      sizeof(float) *
                      MICEX_SOUND_SAMPLES_BUF_SIZE);

    xTaskCreate(pv_processor_task,
                "Processor",
                PROCESSOR_TASK_STACK_SIZE,
                NULL,
                PROCESSOR_TASK_PRIORITY,
                NULL);

    continuous_adc_init(channel,
                        sizeof(channel) / sizeof(adc_channel_t),
                        &handle);

    ESP_ERROR_CHECK(
        adc_continuous_register_event_callbacks(handle,
                                                &cbs,
                                                NULL));

    ESP_ERROR_CHECK(adc_continuous_start(handle));

    /* *****************************************************************
     * LOOP PRINCIPAL
     * *****************************************************************/
    while (1) {

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {

            ret = adc_continuous_read(handle,
                                      result,
                                      MICEX_ADC_FRAME_SIZE,
                                      &ret_num,
                                      0);

            if (ret == ESP_OK) {

                parse_ret =
                    adc_continuous_parse_data(handle,
                                              result,
                                              ret_num,
                                              parsed_data,
                                              &num_parsed_samples);

                if (parse_ret == ESP_OK) {

                    for (int i = 0;
                         i < num_parsed_samples;
                         i++) {

                        sound_samp_buf_ADC[sb_count] =
                            (float)parsed_data[i].raw_data;

                        sb_count++;

                        if (sb_count ==
                            MICEX_SOUND_SAMPLES_BUF_SIZE) {

                            /* pré-processamento */
                            preprocess_buffer(
                                sound_samp_buf_ADC,
                                MICEX_SOUND_SAMPLES_BUF_SIZE);

                            xQueueSend(XQ,
                                       (void *)sound_samp_buf_ADC,
                                       0);

                            sb_count = 0;
                        }
                    }

                } else {

                    ESP_LOGE(TAG,
                             "Data parsing failed: %s",
                             esp_err_to_name(parse_ret));
                }

                vTaskDelay(1);

            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    /* Clean up se saísse do loop */
    adc_continuous_stop(handle);
    adc_continuous_deinit(handle);
}

/* *****************************************************************
 * TASK DE PROCESSAMENTO (Implementação Completa do Ponto 3 - Debounce e Trava)
 * *****************************************************************/
void pv_processor_task(void *pvParam)
{
    float *sound_samp_buf_proc =
        heap_caps_malloc(sizeof(float) *
                         MICEX_SOUND_SAMPLES_BUF_SIZE,
                         MALLOC_CAP_DMA);

    /* Variáveis estáticas de diagnóstico (Ponto 1) */
    static float peak_energy_0 = 0.0f;
    static float peak_energy_1 = 0.0f;
    static float peak_energy_2 = 0.0f;
    static float peak_rms = 0.0f;
    static int loop_counter = 0;

    /* Variáveis da FSM com suporte às duas sequências dinâmicas */
    static fsm_state_t current_state = STATE_IDLE;
    static int64_t last_state_change_time = 0;

    /* Variáveis do Ponto 3 - Gestão de Tempo, Debounce e Bloqueio de Repetição */
    static int64_t tom_0_start_time = 0;
    static int64_t tom_1_start_time = 0;
    static int64_t tom_2_start_time = 0;
    static int last_registered_tom = -1; // Guarda qual o último tom aceite (-1 = nenhum)

    for (;;) {

        xQueueReceive(XQ,
                      (void *)sound_samp_buf_proc,
                      portMAX_DELAY);

        /* *************************************************************
         * PONTO 2 — RMS DO SINAL
         * *************************************************************/
        float rms =
            calculate_rms(sound_samp_buf_proc,
                          MICEX_SOUND_SAMPLES_BUF_SIZE);

        /* *************************************************************
         * PONTO 3 — FILTRAGEM FIR (Deteção das Frequências Centrais)
         * *************************************************************/
        float energy_0 =
            detect_frequency_energy(sound_samp_buf_proc,
                                    MICEX_SOUND_SAMPLES_BUF_SIZE,
                                    filter_0,
                                    filter_len_0);

        float energy_1 =
            detect_frequency_energy(sound_samp_buf_proc,
                                    MICEX_SOUND_SAMPLES_BUF_SIZE,
                                    filter_1,
                                    filter_len_1);

        float energy_2 =
            detect_frequency_energy(sound_samp_buf_proc,
                                    MICEX_SOUND_SAMPLES_BUF_SIZE,
                                    filter_2,
                                    filter_len_2);

        /* Registos de pico máximos para calibração */
        if (rms > peak_rms) peak_rms = rms;
        if (energy_0 > peak_energy_0) peak_energy_0 = energy_0;
        if (energy_1 > peak_energy_1) peak_energy_1 = energy_1;
        if (energy_2 > peak_energy_2) peak_energy_2 = energy_2;

        int64_t current_time = esp_timer_get_time();

        /* Booleans instantâneos de validação de tom baseados nos limiares */
        bool tom_0_detectado = (rms > MIN_SIGNAL_RMS) && (energy_0 > DETECTION_THRESHOLD) && ((energy_0 / rms) > RATIO_THRESHOLD);
        bool tom_1_detectado = (rms > MIN_SIGNAL_RMS) && (energy_1 > DETECTION_THRESHOLD) && ((energy_1 / rms) > RATIO_THRESHOLD);
        bool tom_2_detectado = (rms > MIN_SIGNAL_RMS) && (energy_2 > DETECTION_THRESHOLD) && ((energy_2 / rms) > RATIO_THRESHOLD);

        /* *************************************************************
         * LÓGICA DE DEBOUNCE (PONTO 3 - Medição da estabilidade temporal)
         * *************************************************************/
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

        /* Booleans finais pós-debounce: O tom tem de estar ativo há pelo menos DEBOUNCE_TIME_US */
        bool tom_0_estavel = (tom_0_start_time > 0) && ((current_time - tom_0_start_time) >= DEBOUNCE_TIME_US);
        bool tom_1_estavel = (tom_1_start_time > 0) && ((current_time - tom_1_start_time) >= DEBOUNCE_TIME_US);
        bool tom_2_estavel = (tom_2_start_time > 0) && ((current_time - tom_2_start_time) >= DEBOUNCE_TIME_US);

        /* *************************************************************
         * CONTROLO DE LIBERTAÇÃO DA NOTA (PONTO 3)
         * Se o último tom registado deixou de ser ouvido, desbloqueia a FSM
         * *************************************************************/
        if (last_registered_tom == 0 && !tom_0_detectado) last_registered_tom = -1;
        if (last_registered_tom == 1 && !tom_1_detectado) last_registered_tom = -1;
        if (last_registered_tom == 2 && !tom_2_detectado) last_registered_tom = -1;

        /* *************************************************************
         * AVALIAÇÃO GLOBAL DE TIMEOUT
         * Se a máquina não estiver em IDLE e exceder o tempo limite, faz reset
         * *************************************************************/
        if (current_state != STATE_IDLE && (current_time - last_state_change_time) > SEQUENCE_TIMEOUT_US) {
            current_state = STATE_IDLE;
            last_registered_tom = -1;
            printf("\n>>> [FSM] Timeout excedido entre tons! Reset para IDLE.");
        }

        /* *************************************************************
         * MÁQUINA DE ESTADOS - FSM (Gatilhada apenas por Notas Estáveis e Novas)
         * *************************************************************/
        switch (current_state) {
            
            case STATE_IDLE:
                if (tom_0_estavel && last_registered_tom != 0) {
                    current_state = STATE_OPEN_W_1;
                    last_state_change_time = current_time;
                    last_registered_tom = 0;
                    printf("\n>>> [FSM] Nota estável [0] aceite! Sequência de Abertura Iniciada... (Espera Tom 1)");
                } else if (tom_2_estavel && last_registered_tom != 2) {
                    current_state = STATE_CLOSE_W_1;
                    last_state_change_time = current_time;
                    last_registered_tom = 2;
                    printf("\n>>> [FSM] Nota estável [2] aceite! Sequência de Fecho Iniciada... (Espera Tom 1)");
                }
                break;

            /* ------- FLUXO DA SEQUÊNCIA DE ABERTURA (0 -> 1 -> 2 -> 1) ------- */
            case STATE_OPEN_W_1:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_OPEN_W_2;
                    last_state_change_time = current_time;
                    last_registered_tom = 1;
                    printf("\n>>> [FSM] Nota estável [1] aceite! Sequência Abertura: [0 -> 1]... (Espera Tom 2)");
                }
                break;

            case STATE_OPEN_W_2:
                if (tom_2_estavel && last_registered_tom != 2) {
                    current_state = STATE_OPEN_W_1_FINAL;
                    last_state_change_time = current_time;
                    last_registered_tom = 2;
                    printf("\n>>> [FSM] Nota estável [2] aceite! Sequência Abertura: [0 -> 1 -> 2]... (Espera Tom 1 final)");
                }
                break;

            case STATE_OPEN_W_1_FINAL:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_IDLE; 
                    last_registered_tom = 1;
                    gpio_set_level(LED_GPIO_PIN, 1); // LIGA LED (Abre Porta)
                    printf("\n>>> [FSM] !!! SEQUÊNCIA DE ABERTURA COMPLETA (0121) !!! LED LIGADO (ON)");
                }
                break;

            /* ------- FLUXO DA SEQUÊNCIA DE FECHO (2 -> 1 -> 0 -> 1) ------- */
            case STATE_CLOSE_W_1:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_CLOSE_W_0;
                    last_state_change_time = current_time;
                    last_registered_tom = 1;
                    printf("\n>>> [FSM] Nota estável [1] aceite! Sequência Fecho: [2 -> 1]... (Espera Tom 0)");
                }
                break;

            case STATE_CLOSE_W_0:
                if (tom_0_estavel && last_registered_tom != 0) {
                    current_state = STATE_CLOSE_W_1_FINAL;
                    last_state_change_time = current_time;
                    last_registered_tom = 0;
                    printf("\n>>> [FSM] Nota estável [0] aceite! Sequência Fecho: [2 -> 1 -> 0]... (Espera Tom 1 final)");
                }
                break;

            case STATE_CLOSE_W_1_FINAL:
                if (tom_1_estavel && last_registered_tom != 1) {
                    current_state = STATE_IDLE;
                    last_registered_tom = 1;
                    gpio_set_level(LED_GPIO_PIN, 0); // DESLIGA LED (Fecha Porta)
                    printf("\n>>> [FSM] !!! SEQUÊNCIA DE FECHO COMPLETA (2101) !!! LED DESLIGADO (OFF)");
                }
                break;
        }

        /* Impressão cíclica de relatórios para monitorização e debug */
        loop_counter++;
        if (loop_counter >= 12) {
            printf("\n=================================================");
            printf("\n        SISTEMA COGNITIVO COM DEBOUNCE (P3)      ");
            printf("\n=================================================");
            const char* state_str = (current_state == STATE_IDLE) ? "IDLE (Espera Tom 0 ou 2)" :
                                    (current_state == STATE_OPEN_W_1) ? "ABERTURA: Espera 1" :
                                    (current_state == STATE_OPEN_W_2) ? "ABERTURA: Espera 2" :
                                    (current_state == STATE_OPEN_W_1_FINAL) ? "ABERTURA: Espera 1 Final" :
                                    (current_state == STATE_CLOSE_W_1) ? "FECHO: Espera 1" :
                                    (current_state == STATE_CLOSE_W_0) ? "FECHO: Espera 0" : "FECHO: Espera 1 Final";
            printf("\n[FSM Estado]   %s", state_str);
            printf("\n[FSM Trava]    Bloqueado na nota: %d (-1 = livre)", last_registered_tom);
            printf("\n[RMS Global]   Atual: %.4f  | Histórico Max: %.4f", rms, peak_rms);
            printf("\n[FREQ_0: 500Hz]  Atual: %.4f  | Estável? %s", energy_0, tom_0_estavel ? "SIM" : "NAO");
            printf("\n[FREQ_1: 1340Hz] Atual: %.4f  | Estável? %s", energy_1, tom_1_estavel ? "SIM" : "NAO");
            printf("\n[FREQ_2: 2180Hz] Atual: %.4f  | Estável? %s", energy_2, tom_2_estavel ? "SIM" : "NAO");
            printf("\n=================================================\n");
            loop_counter = 0;
        }
    }
}

/* *************************************************************** * ADC CALLBACK
 * *****************************************************************/
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t mustYield = pdFALSE;

    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/* *************************************************************** * ADC INIT
 * *****************************************************************/
static void continuous_adc_init(adc_channel_t *channel,
                                 uint8_t channel_num,
                                 adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = MICEX_ADC_BUF_SIZE,
        .conv_frame_size = MICEX_ADC_FRAME_SIZE,
    };

    ESP_ERROR_CHECK(
        adc_continuous_new_handle(&adc_config,
                                  &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = MICEX_ADC_SAMPLE_FREQ,
        .conv_mode = MICEX_ADC_CONV_MODE,
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

    ESP_ERROR_CHECK(
        adc_continuous_config(handle,
                              &dig_cfg));

    *out_handle = handle;
}