 /* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF    2450        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define NO_OF_VALUES   40          //RMS



static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel2 = ADC_CHANNEL_7;
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;


static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void app_main(void)
{

   //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    float energia = 0;
    float custo = 0;
    float somatorio = 0;
    float potencia = 0;
   

    while (1) {
        uint32_t adc_reading = 0;
        uint32_t adc_reading2 = 0;
        float voltage_rms = 0;
        float corrente_rms = 0;
        float Vrms = 0;
        float Irms = 0;
        uint32_t pos = 0;
        uint32_t neg = 0;

        for (int j = 0; j < NO_OF_VALUES; j++) {
            //int64_t t1 = esp_timer_get_time();
        //TENSAO ELETRICA

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        voltage -= 950;
        voltage *= 0.660;
        //voltage -= 985;
        //voltage *= 0.644;
        voltage_rms += voltage*voltage;
        if (voltage>0) {
            pos += 1;
        } else {
            neg += 1;
        }
 
        printf("Raw: %d\tVoltage: %fV\n", adc_reading, voltage);
        vTaskDelay(pdMS_TO_TICKS(200));
        


        //CORRENTE ELETRICA

        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading2 += adc1_get_raw((adc1_channel_t)channel2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, width, &raw);
                adc_reading2 += raw;
            }
        }
        adc_reading2 /= NO_OF_SAMPLES;

        //Convert adc_reading to corrente in mA
        float corrente = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars);
        corrente -= 1555;
        corrente *= 0.0041;
        //corrente -= 1585;
        //corrente *= 0.0041;
        corrente_rms += corrente*corrente;
        somatorio += abs(voltage*corrente);
        printf("Raw: %d\tCorrente: %f\n", adc_reading2, corrente);
        vTaskDelay(pdMS_TO_TICKS(1000));
        }

        //CALCULO RMS

        //Calculo RMS Tensao
        voltage_rms /= NO_OF_VALUES;
        Vrms = sqrt(voltage_rms);

        //Calculo RMS Corrente
        corrente_rms /= NO_OF_VALUES;
        Irms = sqrt(corrente_rms);

        //POTENCIA E ENERGIA E CUSTO R$
        potencia = somatorio/NO_OF_VALUES;
        energia += potencia/3600;
        custo += 0.7358*energia;

        //Print do RMS da tensao e corrente
        printf("Tensão RMS: %fV\n", Vrms);
        printf("Corrente RMS: %fI\n", Irms);
        printf("neg: %d\tpos: %d\n", neg, pos);
        printf("Energia consumida: %fWh\n", energia);
        printf("Custo R$: %f\n", custo);
        voltage_rms = 0;
        corrente_rms = 0;
        somatorio = 0;
        potencia = 0;
    }
}
