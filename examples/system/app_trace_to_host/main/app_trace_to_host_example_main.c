/* Application Trace to Host Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_trace.h"
#include "esp_log.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/dac.h"
#include "soc/adc_channel.h"
#include "soc/dac_channel.h"

#define ADC1_TEST_CHANNEL (ADC_CHANNEL_6)

#define TEST_SAMPLING_PERIOD 20

/*
 * When setting custom divider of RTC 8 MHz clock in menuconfig,
 * use the following values to set the CW frequency:
 * ~ 50 Hz (entered below)
 *     RTC_CLK_8M_DIV 7
 *     CW_FREQUENCY_STEP 3
 * ~ 60 Hz
 *     RTC_CLK_8M_DIV 1
 *     CW_FREQUENCY_STEP 1
 */
#ifdef CONFIG_CUSTOM_RTC_CLK_8M_DIV
#define RTC_CLK_8M_DIV 7
#define CW_FREQUENCY_STEP 3
#else
#define CW_FREQUENCY_STEP 1
#endif

static const char *TAG = "example";

/*
 * Enable cosine waveform generator (CW)
 * on DAC channel 1 to provide sinusoidal signal
 * It can be used instead of a live signal for testing
 * of speed of logging to the host
 * sequentially with data retrieval from ADC
 */
static void enable_cosine_generator(void)
{
    // Enable tone generator common to both DAC channels 1 and 2
    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
    // Enable / connect tone tone generator on / to channel 1
    SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN1_M);
    // Invert MSB, otherwise part of the waveform will be inverted
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV1_S);
    // Set the frequency of waveform on CW output
#ifdef CONFIG_CUSTOM_RTC_CLK_8M_DIV
    REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, RTC_CLK_8M_DIV);
    ESP_LOGI(TAG, "Custom divider of RTC 8 MHz clock has been set.");
#endif
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, CW_FREQUENCY_STEP, SENS_SW_FSTEP_S);

    dac_output_enable(DAC_CHANNEL_1);
}

/*
 * Sample data an ADC1 channel 6
 * over specific 'sampling_period' in milliseconds.
 * Print out sampling result using standard ESP_LOGI() function.
 * Return the number of samples collected.
 */
static int adc1_sample_and_show(adc_oneshot_unit_handle_t adc1_handle, int sampling_period)
{
    int i = 0;
    uint32_t sampling_start = esp_log_timestamp();
    do {
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC1_TEST_CHANNEL, &adc_raw));
        ESP_LOGI(TAG, "Sample:%d, Value:%d", ++i, adc_raw);
    } while (esp_log_timestamp() - sampling_start < sampling_period);
    return i;
}

/*
 * Main program loop that is sampling data on ADC
 * and logging results with application tracing to the host
 * as well as for comparison printing out sampling result to UART
 */
void app_main(void)
{
    ESP_LOGI(TAG, "Enabling ADC1 on channel 6 / GPIO%d.", ADC1_CHANNEL_6_GPIO_NUM);

    //-------------ADC1 Init---------------//
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    //-------------ADC1 Channel Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC1_TEST_CHANNEL, &config));

    ESP_LOGI(TAG, "Enabling CW generator on DAC channel 1 / GPIO%d.", DAC_CHANNEL_1_GPIO_NUM);
    enable_cosine_generator();

    while (1) {
        /*
         * Logging with the Application Trace
         */
        ESP_LOGI(TAG, "Sampling ADC and sending data to the host...");
        // Route LOGx() to the host
        esp_log_set_vprintf(esp_apptrace_vprintf);
        int samples_collected = adc1_sample_and_show(adc1_handle, TEST_SAMPLING_PERIOD);
        // Route LOGx() back to UART
        esp_log_set_vprintf(vprintf);
        // Flush collected data to the host
        esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, 100000);
        ESP_LOGI(TAG, "Collected %d samples in %d ms.\n", samples_collected, TEST_SAMPLING_PERIOD);

        /*
         * Logging to UART
         */
        ESP_LOGI(TAG, "Sampling ADC and sending data to the UART...");
        samples_collected = adc1_sample_and_show(adc1_handle, TEST_SAMPLING_PERIOD);
        ESP_LOGI(TAG, "Collected %d samples in %d ms.\n", samples_collected, TEST_SAMPLING_PERIOD);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
}
