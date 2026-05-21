/* ********************************************************************************************************************************* 
Teste 
* Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonecsa, Luis Moutinho 2026/Apr.
 * 
 * Tested:
 *  ESP32-C6 DevKitC-1
 * 
 * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, to allow higher frequencies
 * - Signal is processed by a Band-Pass filter, in the time-domain, to identify defined frequencies 
 *  
 * Microphone is a MEMS Adafruit Silicon MEMS Microphone Breakout - SPW2430.
 *     Supplied with 3.3-5V, output at DC pin has a 0.7 V and a 100 mVpp "when talking near". 
 *      In my case I had around 1 V. So the attenuation cannot be 0 dB. 
 *      I have used 2.5 dB (vref/0.7), to get 1.3 to 1.5 volts for Vref+ and avoid saturation
 *      Check other mics to see if this is normal.  
 * 
 *  
 * Bibliography: 
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_continuous.html 
 *      https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-library.html      
 * 
 * Based on the sample code  provided by EspressIF:
 *      https://github.com/espressif/esp-idf/tree/47faecc3/examples/peripherals/adc/continuous_read 
 * 
 * NOTE: must run idf.py add-dependency "espressif/esp-dsp" when creating a new project using dsp functionality
 ***********************************************************************************************************************************/ 

/* ********************************* 
 * Includes
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

/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH

#define MICEX_ADC_FRAME_SIZE             512
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)
#define MICEX_ADC_SAMPLE_FREQ            (20 * 1000)

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048
#define MAX_FILT_IR_LEN                 200

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};
static TaskHandle_t s_task_handle;

static const char *TAG = "MIC_EXAMPLE";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES];

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192
#define PROCESSOR_TASK_PRIORITY         ( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;

/* Impulse response filter and related variables */
__attribute__((aligned(16))) float hbpf2k[] = {
    /* (mantido o teu array completo sem alterações) */
};

/* *************************************************************** 
 * PREPROCESSAMENTO DO SINAL (PONTO 1)
 * Remove offset DC e normaliza amplitude
 *****************************************************************/
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
        if (a > max_abs) max_abs = a;
    }

    /* normalização simples */
    if (max_abs > 0.0f) {
        float scale = 1.0f / max_abs;
        for (int i = 0; i < n; i++) {
            buf[i] *= scale;
        }
    }
}

/* *************************************************************** 
 * Function prototypes 
 *****************************************************************/
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
static void pv_processor_task(void *pvParam);

/******************************************************************* 
 * The main task 
 *******************************************************************/
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

    memset(result, 0x00, MICEX_ADC_FRAME_SIZE);

    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);

    s_task_handle = xTaskGetCurrentTaskHandle();

    cbs.on_conv_done = s_conv_done_cb;
    cbs.on_pool_ovf = NULL;

    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    XQ = xQueueCreate(1, sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE);
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL);

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle);
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle));

    /* *****************************************************************
     * LOOP PRINCIPAL: CAPTURA → CONVERSÃO → PREPROCESSAMENTO → ENVIAR
     *****************************************************************/
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);

            if (ret == ESP_OK) {

                parse_ret = adc_continuous_parse_data(handle, result, ret_num,
                                                     parsed_data, &num_parsed_samples);

                if (parse_ret == ESP_OK) {

                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float)parsed_data[i].raw_data;
                        sb_count++;

                        if (sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) {

                            /* PONTO 1: PREPROCESSAMENTO DO BUFFER */
                            preprocess_buffer(sound_samp_buf_ADC,
                                              MICEX_SOUND_SAMPLES_BUF_SIZE);

                            xQueueSend(XQ, (void *)sound_samp_buf_ADC, 0);
                            sb_count = 0;
                        }
                    }

                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }

                vTaskDelay(1);

            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

/* *****************************************************************
 * TASK DE PROCESSAMENTO
 *****************************************************************/
void pv_processor_task(void *pvParam)
{
    float *sound_samp_buf_proc =
        heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);

    for (;;) {
        xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY);

        printf("\nFirst 100 samples of the sound frame (preprocessed):----------- ");

        for (int n = 0; n < 100; n++) {
            if (n % 10 == 0) {
                printf("\n[%d to %d]:", n, n + 9);
            }
            printf("%8.2f ", sound_samp_buf_proc[n]);
        }

        printf("\n---------------------\n");
    }
}

/* *************************************************************** 
 * ADC CALLBACK
 *****************************************************************/
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                      const adc_continuous_evt_data_t *edata,
                                      void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

/* *************************************************************** 
 * ADC INIT
 *****************************************************************/
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