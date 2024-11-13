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
#include "driver/timer.h"

#include <string.h> //Requires by memset
#include "esp_system.h"
#include "esp_spi_flash.h"
#include <esp_http_server.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/api.h>
#include <lwip/netdb.h>

#define DEFAULT_VREF    2450        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   16          //Multisampling
#define NO_OF_VALUES    25          //Coletas



static const char *TAG = "espressif"; // TAG for debug

float energia = 0;
float custo = 0;
float somatorio = 0;
float potencia = 0;

uint32_t adc_reading = 0;
uint32_t adc_reading2 = 0;
float voltage_rms = 0;
float corrente_rms = 0;
float Vrms = 0;
float Irms = 0;
uint32_t pos = 0;
uint32_t neg = 0;
uint32_t loop = 0;

float tensao[NO_OF_VALUES] = {};
uint32_t corrente[NO_OF_VALUES] = {};
float dados_energia[][4] = {};
uint8_t flag = 0;


static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
//static const adc_channel_t channel2 = ADC_CHANNEL_7;    //GPIO35 if ADC1, GPIO27 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12; // Resolução do ADC em 12 bits - (0 - 4095)
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel = ADC_CHANNEL_6;     // GPIO7 if ADC1, GPIO17 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;      // Escolha da atenuação do conversor. A atenuação DB_11 permite que o sinal de entrada do ADC alcance valores maiores (2450mV)
static const adc_unit_t unit = ADC_UNIT_1;             // Escolha do conversor ADC


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


// Função principal a ser processada no core_1: Conversão AD do sinal de tensão e corrente

void TaskRunningOnAppCore(void *arg) {
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC - O ADC escolhido foi o unit_1 com resolução 12 bits (width), channel 07 (GPIO34) e attenuation DB_11
    if (unit == ADC_UNIT_1) {
        adc1_config_width(width);   // Configuração da resolução da saída do ADC - 12 bits
        adc1_config_channel_atten(channel, atten);   // Configuração da atenuação do ADC - DB_11
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC - Caracteriza o ADC com uma atenuação específica
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
    
    loop = 0;
 
    while (loop<5) {   

    // NO_OF_VALUES - Número de amostras que serão utilizadas para o cálculo do RMS
    // Loop de aquisição de dados na entrada analógica, tanto de tensão (Pino 34) quanto de corrente (pino 35).
    // Nesse loop, para cada amostra coletada são realizadas NO_OF_SAMPLES leituras no canal analógico e em seguida é tirada uma média.
    // Essa técnica, chamada de Multisampling, reduz a relevância dos ruídos. Cada uma das NO_OF_VALUES é resultado dessa média de coletas. 
    // Esses valores são armazenados em um vetor. Esse fluxo serve tanto para a tensão quanto para corrente. Apenas quando todos os NO_OF_VALUES
    // de tensão e corrente são armazenados nos vetores é que os valores coletados são transformados em mV.
    // Isso foi realizado pois a conversão em mV leva mais tempo e estava comprometendo as 20 coletas por ciclo da senoide.
        printf("Core %d!\n", xPortGetCoreID() );
        printf("XYZ\n");
        printf("loop:%d\n",loop);
        for (int j = 0; j < NO_OF_VALUES; j++) {

        //TENSAO ELETRICA
        
        //Multisampling - Realiza (NO_OF_SAMPLES) medições em sequência para minimizar os efeitos de ruídos.
        // É calculada a média dessas medições, que contará como apenas uma conversão AD
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)channel);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)channel, width, &raw);
                    adc_reading += raw;
                }
            } // Fim do Multisampling

            adc_reading /= NO_OF_SAMPLES;   // Obtém a média das medições 
        
        //Convert adc_reading to voltage in mV
            float voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
            tensao[j] = voltage/1000;        // Guarda cada um dos NO_OF_VALUES obtidos na conversão AD
        //printf("Valor da tensao: %f\n",tensao[j]);
        //voltage -= 2600; //Tirar o OFFSET
        //voltage *= 0.001;
        //voltage *= 0.660;
        //voltage_rms += voltage*voltage;
        
        //printf("%f\n", voltage/1000);
        //printf("Raw: %d\tVoltage: %fV\n", , voltage);
        //vTaskDelay(pdMS_TO_TICKS(12));

        }
        loop += 1;
        flag = 1;
    }
}

void app_main(void)
{
    printf("Core %d!!!!\n**", xPortGetCoreID() );
    printf("ABC\n");


    xTaskCreatePinnedToCore(TaskRunningOnAppCore, 
                        "TaskOnApp", 
                        2048, 
                        NULL, 
                        4, 
                        NULL,
                         1);

        //CALCULO RMS

        //Calculo RMS Tensao
    if (flag == 1) {
        for (int s = 0; s < NO_OF_VALUES; s++) {
                //voltage = esp_adc_cal_raw_to_voltage(tensao[s], adc_chars); //Convert adc_reading to voltage in mV
                //voltage -= 2.2; // Retirada do OFFSET gerado pelo módulo ZMPT101B
                //voltage *= 622.5; // Relação de transformação da tensão (221V/0,355V). 221V - Valor real (multímetro); 0,355V valor medido na saída do ZMPT pelo osciloscópio
            voltage_rms += tensao[s]*tensao[s];        // Guarda cada um dos NO_OF_VALUES obtidos na conversão AD para transformar para mV apenas após a aquisição de todos os NO_OF_VALUES
            //printf("%f\n", voltage);
                //printf("Valor RMS calculado: %d\n", voltage);

    	    }
            voltage_rms =sqrt(voltage_rms/NO_OF_VALUES);
            //voltage_rms *= 622.5;
            printf("Valor RMS calculado: %d\n", voltage_rms);
        //voltage_rms /= NO_OF_VALUES;
        //Vrms = sqrt(voltage_rms);

        //Calculo RMS Corrente
        //corrente_rms /= NO_OF_VALUES;
        //Irms = sqrt(corrente_rms);

        //POTENCIA E ENERGIA E CUSTO R$
        //potencia = somatorio/NO_OF_VALUES;
        //energia += potencia/(3600);
        //custo += (0.7358*energia)/(900000);

        //Armazenar dados na matriz dados_energia na seguinte ordem de colunas: [Vrms][Irms][potencia][energia]
        //dados_energia[loop][0]=Vrms;
        //dados_energia[loop][1]=Irms;
        //dados_energia[loop][2]=potencia;
        //dados_energia[loop][3]=energia;

        //Print do RMS da tensao e corrente
        //printf("Tensão RMS: %fV\n", Vrms);
        //printf("Corrente RMS: %fI\n", Irms);
        //printf("neg: %d\tpos: %d\n", neg, pos);
        //printf("Energia consumida: %fWh\n", energia);
        //printf("Custo R$: %f\n\n\n\n", custo);
        //voltage_rms = 0;
        //corrente_rms = 0;
        //somatorio = 0;
        //potencia = 0;
        //loop += 1;
    }
}

//Salvar Dados (Vrms, Irms, Potência, Energia) em arquivo .csv - Salvo apenas ao final do processo de leitura
//salvar_em_CSV("dados_energia_coletados.csv", loop, 4, dados_energia);
//int incremento = 0;
//while(1){
//    ESP_LOGI(TAG, "Writing %d", incremento);
//    incremento += 1;

//}

