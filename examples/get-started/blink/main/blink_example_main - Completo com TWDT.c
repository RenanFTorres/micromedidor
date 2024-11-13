
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// #include para ADS1115
#include <esp_log.h>
#include <driver/i2c.h>
#include "ADS1115.h"

// #define para esp32/adc/dual core
#define core_0 0
#define core_1 1

#define DEFAULT_VREF    2450        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   16          //Multisampling
#define NO_OF_VALUES    114          //Coletas

// #define para ads1115
#define SDA_IO (21)                      /*!< gpio number for I2C master data  */
#define SCL_IO (22)                      /*!< gpio number for I2C master clock */

#define FREQ_HZ (1000000)                 /*!< I2C master clock frequency */
#define TX_BUF_DISABLE (0)               /*!< I2C master doesn't need buffer */
#define RX_BUF_DISABLE (0)               /*!< I2C master doesn't need buffer */

#define I2C_NUM I2C_NUM_0                /*!< I2C number */
#define I2C_MODE I2C_MODE_MASTER         /*!< I2C mode to act as */
#define I2C_RX_BUF_STATE RX_BUF_DISABLE  /*!< I2C set rx buffer status */
#define I2C_TX_BUF_STATE TX_BUF_DISABLE  /*!< I2C set tx buffer status */
#define I2C_INTR_ALOC_FLAG (0)           /*!< I2C set interrupt allocation flag */

#define NO_OF_VALUES_ADS 240  

// Declaração de variáveis para esp32/adc/dual core
uint32_t nucleo = 0;
uint32_t contagem = 0;

float energia = 0;
float custo = 0;
float somatorio = 0;
float potencia = 0;
float voltage = 0;

uint32_t adc_reading = 0;
uint32_t adc_reading2 = 0;
float voltage_rms = 0;
float Vrms = 0;
float Irms = 0;
uint32_t pos = 0;
uint32_t neg = 0;
uint8_t flag = 0;

float tensao[NO_OF_VALUES] = {};
float vetor_tensao[NO_OF_VALUES] = {};
float dados_energia[][4] = {};

// Declaração de variáveis para ads1115
static const char *TAG = "Main";
float corrente_rms = 0;
float corrente[NO_OF_VALUES_ADS] = {};
float vetor_corrente[NO_OF_VALUES_ADS] = {};
uint8_t flag2 = 0;

// Configuração do esp32
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

// Configuração ADS1115

/* i2c setup ----------------------------------------- */
// Config profile for espressif I2C lib
i2c_config_t i2c_cfg = {                     
  .mode = I2C_MODE_MASTER, 
  .sda_io_num = SDA_IO,
  .scl_io_num = SCL_IO,
  .sda_pullup_en = GPIO_PULLUP_DISABLE,
  .scl_pullup_en = GPIO_PULLUP_DISABLE,
  .master.clk_speed = FREQ_HZ,
};

/* ADS1115 setup ------------------------------------- */
// Below uses the default values speficied by the datasheet
ads1115_t ads1115_cfg = {
  .reg_cfg =  ADS1115_CFG_LS_COMP_MODE_TRAD | // Comparator is traditional
              ADS1115_CFG_LS_COMP_LAT_NON |   // Comparator is non-latching
              ADS1115_CFG_LS_COMP_POL_LOW |   // Alert is active low
              ADS1115_CFG_LS_COMP_QUE_DIS |   // Compator is disabled
              ADS1115_CFG_LS_DR_860SPS |     // No. of samples to take
              ADS1115_CFG_MS_MODE_CON |        // Mode is set to continuous
              ADS1115_CFG_MS_PGA_FSR_6_144V,  // Full-scale range of ADC scaling
  .dev_addr = 0x48,
};

// Request single ended on pin AIN1  




// Declaração da função que executa a conversão dos dados na entrada analógica do ads1115
void xADS1115_thred(void *arg) // FreeRTOS task for ADC loop
{
 // Buffer for result
  uint16_t result = 0;
  ADS1115_request_continuous_ended_AIN1();     // all functions except for get_conversion_X return 'esp_err_t' for logging   
  for(;;) 
  {
    // Check conversion state - returns true if conversion is complete 
    //while(!ADS1115_get_conversion_state()) 
    //  vTaskDelay(pdMS_TO_TICKS(5));          // wait 5ms before check again
    
    // Return latest conversion value
    for (int j = 0; j < NO_OF_VALUES_ADS; j++) {
        result = ADS1115_get_conversion();
        corrente[j] = result*6.144/((1L<<15) - 1);   
        //ESP_LOGD(TAG,"Conversion Value: %d", result);
        //printf("Conversion Value: %d and voltage: %f\n", result, corrente[j]);  
        //printf("%f\n",(corrente[j]-0.6));
        result = 0;
    }
    flag2 = 1;
  }
  //ESP_LOGD(TAG,"Should not reach here!");
  //vTaskDelete(NULL);  
}

// Declaração da função que é executada em loop no core 1 do esp32
void hello_task_core_1(void *pvParameter)
{
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

	while(1){    
        //nucleo += 1;
	    //printf("Hello world from CORE %d, nucleo = %d!\n", xPortGetCoreID(), nucleo);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //fflush(stdout);


        // NO_OF_VALUES - Número de amostras que serão utilizadas para o cálculo do RMS
        // Loop de aquisição de dados na entrada analógica, tanto de tensão (Pino 34) quanto de corrente (pino 35).
        // Nesse loop, para cada amostra coletada são realizadas NO_OF_SAMPLES leituras no canal analógico e em seguida é tirada uma média.
        // Essa técnica, chamada de Multisampling, reduz a relevância dos ruídos. Cada uma das NO_OF_VALUES é resultado dessa média de coletas. 
        // Esses valores são armazenados em um vetor. Esse fluxo serve tanto para a tensão quanto para corrente. Apenas quando todos os NO_OF_VALUES
        // de tensão e corrente são armazenados nos vetores é que os valores coletados são transformados em mV.
        // Isso foi realizado pois a conversão em mV leva mais tempo e estava comprometendo as 20 coletas por ciclo da senoide.

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
            voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars); //Convert adc_reading to voltage in mV
            tensao[j] = voltage;
    	}
        flag = 1;
    }
}

// Declaração da função que é executada em loop no core 0 do esp32
void hello_task_core_0(void *pvParameter)
{
    xTaskCreate(xADS1115_thred, "xADS1115_thred",2048, NULL, 3, NULL);
	while(1) {
        //nucleo += 1;
	    //printf("Hello world from CoRe %d, nucleo = %d!\n", xPortGetCoreID(), nucleo);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        //xTaskCreate(xADS1115_thred, "xADS1115_thred",2048, NULL, 3, NULL);
    //fflush(stdout);

        // PROCESSAMENTO DOS DADOS COLETADOS
        if (flag == 1 || flag2 == 1 ) {
            //vTaskDelay(10 / portTICK_PERIOD_MS);
            for (int j = 0; j < NO_OF_VALUES; j++) {
                vetor_tensao[j] = tensao[j];
            }
            for (int j = 0; j < NO_OF_VALUES_ADS; j++) {
                vetor_corrente[j] = corrente[j];
            }
            for (int s = 0; s < NO_OF_VALUES; s++) {
                // 1000 - Converte de mV para V
                // 2.5 - Retirada do OFFSET gerado pelo módulo ZMPT101B
                voltage_rms += (vetor_tensao[s]/1000-2.5)*(vetor_tensao[s]/1000-2.5);
    	    }
            for (int s = 0; s < NO_OF_VALUES_ADS; s++) {
                // 1000 - Converte de mV para V
                // 2.5 - Retirada do OFFSET gerado pelo módulo ZMPT101B
                corrente_rms += (vetor_corrente[s]-0.75)*(vetor_corrente[s]-0.75); // -0.75 - Retirada do offset do sinal de entrada (valor definido em observação ao sinal analógico pelo osciloscópio)
    	    }
            voltage_rms =sqrt(voltage_rms/NO_OF_VALUES);
            voltage_rms *= 622.5; //// Relação de transformação da tensão (221V/0,355V). 221V - Valor real (multímetro); 0,355V valor medido na saída do ZMPT pelo osciloscópio
            corrente_rms = sqrt(corrente_rms/NO_OF_VALUES_ADS);
            corrente_rms *= (5/4); //// 5 - Fator multiplicativo pelo fato do sensor de corrente ser 5A/1V. /4 - O valor coletado da corrente é 4x o valor real, pois o cabo dá 4 voltas no sensor.
            //corrente_rms *= (5); //// Relação de transformação da corrente (5A/1V). 4 - Quantidade de voltas do cabo no sensor de corrente
            potencia = voltage_rms*corrente_rms;
            energia += potencia*(0.2/3600); // Os valores eficazes são calculados a cada 12 ciclos. 1 ciclo dura 16,67ms, logo, os valores eficazes sao coletados durante 200ms.
            printf("Valor RMS da tensão (V): %.2f\n", voltage_rms);
            printf("Valor RMS da corrente (mA): %.4f\n", 1000*corrente_rms);
            printf("Valor da potência ativa (W): %.2f\n", potencia);
            printf("Valor da energia consumida (Wh): %.4f\n", energia);
            vTaskDelay(500 / portTICK_PERIOD_MS);
            //printf("%f\n", voltage_rms);
        flag = 0;
        flag2 = 0;
        voltage_rms = 0;
        corrente_rms = 0;
        potencia = 0;
        //vTaskDelay(15 / portTICK_PERIOD_MS);
        }
        //if (nucleo == 10) {
        //    contagem += 1;
        //    printf("%d\n", contagem);
        //    vTaskDelay(10 / portTICK_PERIOD_MS);
        //    nucleo = 0;
        //}
	}
}

// Tarefa principal do código, que cria as demais tarefas a serem executadas nos cores 0 e 1
void app_main()
{
    //Inicialização ads1115
    ESP_LOGD(TAG,"Starting ADS1115 example..");

    // Setup I2C
    i2c_param_config(I2C_NUM, &i2c_cfg);
    i2c_driver_install(I2C_NUM, I2C_MODE, I2C_RX_BUF_STATE, I2C_TX_BUF_STATE, I2C_INTR_ALOC_FLAG);

    // Setup ADS1115
    ADS1115_initiate(&ads1115_cfg);
  
    
    
    
    // Start ADS loop
    //xTaskCreate(xADS1115_thred, "xADS1115_thred",2048, NULL, 3, NULL);


    //Inicialização das tarefas que serão executadas no core 0 e 1 do esp32
    //nvs_flash_init();
	//xTaskCreatePinnedToCore(&hello_task_core_0, "core1_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_0);
	//xTaskCreatePinnedToCore(&hello_task_core_1, "core0_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_1);
    xTaskCreatePinnedToCore(&hello_task_core_0, "core1_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_0);
	xTaskCreatePinnedToCore(&hello_task_core_1, "core0_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_1);

}