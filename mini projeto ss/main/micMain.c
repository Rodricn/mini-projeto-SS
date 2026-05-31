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
#define MICEX_ADC_SAMPLE_FREQ     (20 * 1000)

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048
#define MAX_FILT_IR_LEN                  200
#define CONV_OUT_LEN                     (MICEX_SOUND_SAMPLES_BUF_SIZE + 41 - 1)

#define LED_GPIO_PIN                     11

#define FREQ_0      500  
#define FREQ_1      1340 
#define FREQ_2      2180 


#define MIN_SIGNAL_RMS           0.030f  // Fica mesmo acima do teu ruído de silêncio (0.022)
#define DETECTION_THRESHOLD      0.008f  // Vai detetar facilmente as tuas leituras que batiam nos 0.010
#define RATIO_THRESHOLD          0.15f   // Muito permissivo, exige apenas 15% de pureza do som

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

/* Alocação Global e Estática com ALIGN16 obrigatório para evitar Guru Meditation Errors no ESP-DSP */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};
__attribute__((aligned(16))) static float sound_samp_buf_ADC[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float sound_samp_buf_proc[MICEX_SOUND_SAMPLES_BUF_SIZE];
__attribute__((aligned(16))) static float conv_out_0[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_1[CONV_OUT_LEN];
__attribute__((aligned(16))) static float conv_out_2[CONV_OUT_LEN];

#define PROCESSOR_TASK_STACK_SIZE       8192
#define PROCESSOR_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4 )

QueueHandle_t XQ;

__attribute__((aligned(16))) float filtro_real_tom_0[] = {
    -0.0034f, -0.0037f, -0.0037f, -0.0030f, -0.0014f,  0.0011f,  0.0044f,  0.0083f,  0.0125f,  0.0166f,
     0.0203f,  0.0232f,  0.0251f,  0.0259f,  0.0255f,  0.0238f,  0.0210f,  0.0173f,  0.0130f,  0.0082f,
     0.0034f,  0.0082f,  0.0130f,  0.0173f,  0.0210f,  0.0238f,  0.0255f,  0.0259f,  0.0251f,  0.0232f,
     0.0203f,  0.0166f,  0.0125f,  0.0083f,  0.0044f,  0.0011f, -0.0014f, -0.0030f, -0.0037f, -0.0037f,
    -0.0034f
};

__attribute__((aligned(16))) float filtro_real_tom_1[] = {
    -0.0016f,  0.0035f,  0.0046f, -0.0006f, -0.0054f, -0.0029f,  0.0049f,  0.0081f,  0.0013f, -0.0093f,
    -0.0105f,  0.0001f,  0.0133f,  0.0138f, -0.0015f, -0.0186f, -0.0201f,  0.0003f,  0.0264f,  0.0355f,
     0.0217f,  0.0355f,  0.0264f,  0.0003f, -0.0201f, -0.0186f, -0.0015f,  0.0138f,  0.0133f,  0.0001f,
    -0.0105f, -0.0093f,  0.0013f,  0.0081f,  0.0049f, -0.0029f, -0.0054f, -0.0006f,  0.0046f,  0.0035f,
    -0.0016f
};

__attribute__((aligned(16))) float filtro_real_tom_2[] = {
     0.0033f,  0.0005f, -0.0049f, -0.0040f,  0.0030f,  0.0061f, -0.0010f, -0.0082f, -0.0019f,  0.0093f,
     0.0053f, -0.0098f, -0.0100f,  0.0085f,  0.0159f, -0.0051f, -0.0226f, -0.0011f,  0.0289f,  0.0125f,
    -0.0336f,  0.0125f,  0.0289f, -0.0011f, -0.0226f, -0.0051f,  0.0159f,  0.0085f, -0.0100f, -0.0098f,
     0.0053f,  0.0093f, -0.0019f, -0.0082f, -0.0010f,  0.0061f,  0.0030f, -0.0040f, -0.0049f,  0.0005f,
     0.0033f
};

float *filter_0 = filtro_real_tom_0;
float *filter_1 = filtro_real_tom_1;
float *filter_2 = filtro_real_tom_2;

int filter_len_0 = sizeof(filtro_real_tom_0) / sizeof(float);
int filter_len_1 = sizeof(filtro_real_tom_1) / sizeof(float);
int filter_len_2 = sizeof(filtro_real_tom_2) / sizeof(float);

static void preprocess_buffer(float *buf, int n)
{
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += buf[i];
    mean /= (float)n;

    /* Escala fixa baseada no hardware ADC (12 bits -> amplitude max 2048) */
    float fixed_scale = 1.0f / 2048.0f;
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

static float detect_frequency_energy(float *input, int input_len, float *filter, int filter_len, float *conv_out)
{
    int out_len = input_len + filter_len - 1;
    memset(conv_out, 0, sizeof(float) * out_len);
    dsps_conv_f32(input, input_len, filter, filter_len, conv_out);
    return calculate_rms(conv_out, out_len);
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
                /* REMOVIDO o vTaskDelay(1) para evitar a perda de frames e falhas no áudio */
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