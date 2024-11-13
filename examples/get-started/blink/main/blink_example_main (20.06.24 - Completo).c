
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
#include <esp_task_wdt.h>

// #include para NTP time
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "esp_wifi.h"

// #define para esp32/adc/dual core
#define core_0 0
#define core_1 1

#define DEFAULT_VREF    2450        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   16          //Multisampling
#define NO_OF_VALUES    114          //Coletas

//#define ADC2_EXAMPLE_CHANNEL    CONFIG_EXAMPLE_ADC2_CHANNEL //#define <nome_da_constante> <valor>
#define ADC2_EXAMPLE_CHANNEL    7 //#define <nome_da_constante> <valor>


// Inicialização WiFi e SNTP
static void obtain_time(void); 
static void initialize_sntp(void);

// Declaração de variáveis para esp32/adc/dual core
uint32_t transporte = 0;
uint32_t contagem = 0;

float energia = 0;
float custo = 0;
float somatorio = 0;
float potencia = 0;
float voltage = 0;
float current = 0;
float tempo = 0;

uint32_t adc_reading = 0;
uint32_t adc2_reading = 0;
float voltage_rms = 0;
float Vrms = 0;
float Irms = 0;
float media_v = 0;
float media_i = 0;
uint32_t pos = 0;
uint32_t neg = 0;
uint8_t flag = 0;

float tensao[NO_OF_VALUES] = {};
float vetor_tensao[NO_OF_VALUES] = {};
float vetor_transporte[NO_OF_VALUES] = {};
float dados_energia[][4] = {};

// Declaração de variáveis para ads1115
static const char *TAG = "Main";
float corrente_rms = 0;
float corrente[NO_OF_VALUES] = {};
float vetor_corrente[NO_OF_VALUES] = {};
uint16_t flag2 = 0;
uint16_t result = 0;

// Declaração de variáveis para NTP
time_t now; 
time_t tempo0; 
time_t tempo1; 
struct tm timeinfo; //Struct com as infomações de data e hora
int flag3 = 0;
int passagem = 0;

#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL); //Atualiza a hora atual imediatamente e armazena na estrutura timeval "tv"
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED); //Define o status da sincronização de horário como completo.
}
#endif


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


float time_diff(struct timeval *start, struct timeval *end) {
    return (end->tv_sec - start->tv_sec) + 1e-6 * (end->tv_usec - start->tv_usec);
    }

struct timeval start;
struct timeval end;


// Declaração da função que é executada em loop no core 1 do esp32
void hello_task_core_1(void *pvParameter)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC - O ADC escolhido foi o unit_1 com resolução 12 bits (width), channel 06 (GPIO34) e attenuation DB_11
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

    //printf("Hello world from COREEEEEEEEE %d!\n", xPortGetCoreID());

	while(1){    
        
        // NO_OF_VALUES - Número de amostras que serão utilizadas para o cálculo do RMS
        // Loop de aquisição de dados na entrada analógica, tanto de tensão (Pino 34) quanto de corrente (pino 35).
        // Nesse loop, para cada amostra coletada são realizadas NO_OF_SAMPLES leituras no canal analógico e em seguida é tirada uma média.
        // Essa técnica, chamada de Multisampling, reduz a relevância dos ruídos. Cada uma das NO_OF_VALUES é resultado dessa média de coletas. 
        // Esses valores são armazenados em um vetor. Esse fluxo serve tanto para a tensão quanto para corrente. Apenas quando todos os NO_OF_VALUES
        // de tensão e corrente são armazenados nos vetores é que os valores coletados são transformados em mV.
        // Isso foi realizado pois a conversão em mV leva mais tempo e estava comprometendo as 20 coletas por ciclo da senoide.
      //if (transporte == 0) {
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
        for (int j = 0; j < NO_OF_VALUES; j++) {
                vetor_transporte[j] = tensao[j];
            }
        flag = 1;
        //transporte = 1;
      //}  
    }
}

// Declaração da função que é executada em loop no core 0 do esp32
void hello_task_core_0(void *pvParameter)
{

    int     read_raw;
    int l;
    float vout;
    esp_err_t r;  // Macro que pode ser usada para identificar erro. Imprime o código do erro, o local do erro e a instrução com falha na saída serial, mas não encerra o programa.

    gpio_num_t adc_gpio_num; // Enumerador do pino GPIO

    r = adc2_pad_get_io_num( ADC2_EXAMPLE_CHANNEL, &adc_gpio_num ); //Obtém o número GPIO de um canal ADC2 específico. ADC2_EXAMPLE_CHANNEL está definido no arquivo sdkconfig.h como 7. No ESP32 O ADC2_CH7 é o GPIO27
    assert( r == ESP_OK ); //assert() testa se uma expressão é verdadeira

    printf("ADC2 channel %d @ GPIO %d.\n", ADC2_EXAMPLE_CHANNEL, adc_gpio_num);

    //be sure to do the init before using adc2.
    printf("adc2_init...\n");
    adc2_config_channel_atten( ADC2_EXAMPLE_CHANNEL, ADC_ATTEN_11db ); //Configura o canal ADC2, incluindo configuração de atenuação.

    printf("start conversion.\n");
    l = 1;

    while(l==1){

        //Início do código do exemplo ADC2
        r = adc2_get_raw( ADC2_EXAMPLE_CHANNEL, width, &read_raw); // Realiza converção no ADC2 com 12 bits e guarda no endereço de memória da variável read_raw
        //vout = read_raw*(2.450/4095);
        if ( r == ESP_OK ) { // Verifica a veracidade de r
            //printf("%d dig, %f Volts\n", read_raw, vout ); // É printado o valor da conversão do ADC2
            l=2;
        } else if ( r == ESP_ERR_INVALID_STATE ) { // Erro de inicialização do ADC2
            printf("%s: ADC2 not initialized yet.\n", esp_err_to_name(r));
        } else if ( r == ESP_ERR_TIMEOUT ) { // Erro por uso de Wi-Fi. ADC2 é usado no Wi-Fi.
            //This can not happen in this example. But if WiFi is in use, such error code could be returned.
            printf("%s: ADC2 is in use by Wi-Fi.\n", esp_err_to_name(r));
        } else {
            printf("%s\n", esp_err_to_name(r));
        }
    //Fim do código do exemplo ADC2

    }

	while(1) {

    
    //printf("Hello world from CoRe %d\n", xPortGetCoreID());
    // Buffer for result
    gettimeofday(&start, NULL);
    //printf("%f s \n",start);
    // Return latest conversion value
         flag2 = 0;
        for (int j = 0; j < NO_OF_VALUES; j++) {
            
            //Multisampling - Realiza (NO_OF_SAMPLES) medições em sequência para minimizar os efeitos de ruídos.
            // É calculada a média dessas medições, que contará como apenas uma conversão AD
//            example_disconnect();
            for (int i = 0; i < NO_OF_SAMPLES; i++) {

                    int raw;
                    adc2_get_raw( ADC2_EXAMPLE_CHANNEL, ADC_WIDTH_BIT_12, &raw);
                    //adc2_get_raw( ADC2_EXAMPLE_CHANNEL, width, &raw);
                    adc2_reading += raw;

            } // Fim do Multisampling
//            example_connect();
            adc2_reading /= NO_OF_SAMPLES;   // Obtém a média das medições 
            current = adc2_reading*(2.450/4095); //Convert adc_reading to voltage in mV
            corrente[j] = current;            
            adc2_reading = 0;
            //ESP_LOGD(TAG,"Conversion Value: %d", result);
            //printf("Conversion Value: %d and voltage: %f\n", result, corrente[j]);  
            //printf("%f\n",(corrente[j]-0.6));
            result = 0;
            flag2 += 1;
        }

        // PROCESSAMENTO DOS DADOS COLETADOS
        if (flag == 1 && flag2 == 0){//NO_OF_VALUES) {
        
            for (int j = 0; j < NO_OF_VALUES; j++) {
                vetor_tensao[j] = vetor_transporte[j]/1000; // 1000 - Converte de mV para V
                media_v += vetor_transporte[j]/1000;
            }
            transporte = 0;
            media_v /=NO_OF_VALUES;
            //printf("Valor médio da tensão: %.4f\n",media_v);
            //media_v = 0;

            for (int j = 0; j < NO_OF_VALUES; j++) {
                media_i += corrente[j];
            }
            media_i /=NO_OF_VALUES;
            
            for (int s = 0; s < NO_OF_VALUES; s++) {
                
                // 2.35 - Retirada do OFFSET gerado pelo módulo ZMPT101B
                voltage_rms += (vetor_tensao[s]-media_v)*(vetor_tensao[s]-media_v);
    	    }
            for (int s = 0; s < NO_OF_VALUES; s++) {
                // -0.75 - Retirada do offset do sinal de entrada (valor definido em observação ao sinal analógico pelo osciloscópio)
                corrente_rms += (corrente[s]-media_i)*(corrente[s]-media_i);
                //vetor_corrente[s] = 1.408*vetor_corrente[s]-media_i;
                //corrente_rms += (vetor_corrente[s])*(vetor_corrente[s]);
    	    }
            
            voltage_rms =sqrt(voltage_rms/NO_OF_VALUES);
            voltage_rms *= 623; //// Relação de transformação da tensão (220V/0,355V). 220V - Valor real (multímetro); 0,355V valor medido na saída do ZMPT pelo osciloscópio
            
            corrente_rms = 1.408*sqrt(corrente_rms/NO_OF_VALUES); 
            //corrente_rms = sqrt(corrente_rms/NO_OF_VALUES_ADS); 
            corrente_rms /= 5;
            corrente_rms *= 5;
            //// 1.408 fator multiplicativo pois o sinal de corrente após o sensor passa por um circuito gerador de offset que atenua o sinal da corrente em 0,71. Logo, para obter o sinal de corrente original deve-se multiplicar a corrente eficaz calculada por 1/0,71 = 1,408
            //// 5 - Fator multiplicativo pelo fato do sensor de corrente ser 5A/1V. /5 - O valor coletado da corrente é 5x o valor real, pois o cabo dá 4 voltas no sensor.
            // corrente_rms *= (5); //// Relação de transformação da corrente (5A/1V). 5 - Quantidade de voltas do cabo no sensor de corrente
            potencia = voltage_rms*corrente_rms;
            gettimeofday(&end, NULL);
            tempo = time_diff(&start, &end);//+0.00035; // tempo em segundos
            tempo /= 3600;
            //printf("%.6f\n",tempo);
            energia += potencia*tempo; // Os valores eficazes são calculados a cada 12 ciclos. 1 ciclo dura 16,67ms, logo, os valores eficazes sao coletados durante 200ms.
            //printf("Valor RMS da tensão (V): %.2f\n", voltage_rms);
            //printf("Valor RMS da corrente (A): %.6f\n", corrente_rms);
            //printf("Valor da potência ativa (W): %.2f\n", potencia);
            //printf("Valor da energia consumida (Wh): %.4f\n", energia);
            time(&now); //Função que retorna o valor do tempo em segundos desde a Epoch
            localtime_r(&now, &timeinfo); //Função que converte o valor do tempo em segundos desde a Epoch contido em "now" e armazena os dados detalhados de tempo em "timeinfo"
            if (timeinfo.tm_sec != passagem) {
                flag3 = 1;
            }
            if ((timeinfo.tm_sec == 00 || (timeinfo.tm_sec > 0 && timeinfo.tm_sec < 60 )) && flag3 == 1) {
                printf("%.4f, %.2d/%.2d/%.2d %.2d:%.2d:%.2d\n", energia,timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year+1900,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec);//Formatação do tempo e impressão como: dia/mês/ano hora:minuto:segundo
                printf("Valor RMS da tensão (V): %.2f\n", voltage_rms);
                //printf("Valor médio da tensão: %.4f\n",media_v);
                printf("Valor RMS da corrente (A): %.4f\n", corrente_rms);
                //printf("%.4f, %.2f, %.4f\n",corrente_rms, voltage_rms,energia);
                passagem = timeinfo.tm_sec;
                flag3 = 0;
                energia = 0;
            }
            //vTaskDelay(500 / portTICK_PERIOD_MS);
            //printf("\n");
            //printf("time spent: %0.8f sec\n", tempo); // time_diff(&start, &end));
        flag = 0;
        flag2 = 0;
        voltage_rms = 0;
        corrente_rms = 0;
        potencia = 0;
        media_v = 0;
        media_i = 0;
        memset(tensao, 0, sizeof(tensao));
        memset(vetor_tensao, 0, sizeof(vetor_tensao));
        memset(vetor_transporte, 0, sizeof(vetor_transporte));
        memset(corrente, 0, sizeof(corrente));
        memset(vetor_corrente, 0, sizeof(vetor_corrente));
        //vTaskDelay(15 / portTICK_PERIOD_MS);

        }
    }
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}


// Tarefa principal do código, que cria as demais tarefas a serem executadas nos cores 0 e 1
void app_main()
{

    time(&now); //Função que retorna o valor do tempo em segundos desde a Epoch
    localtime_r(&now, &timeinfo); //Função que converte o valor do tempo obtido em "now" em informações detalhadas de tempo que são armazenadas em "timeinfo"
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    #ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH //Atualização suave do tempo. O erro de tempo é reduzido gradualmente usando a função adjtime. Se a diferença entre o tempo de resposta do SNTP e o tempo do sistema for grande (mais de 35 minutos), atualize imediatamente.
    else { //Sequência de ações que serão realizadas caso o método escolhido nao seja o SMOOTH
    
        // add 500 ms error to the current system time.
        // Only to demonstrate a work of adjusting method!
        {
            ESP_LOGI(TAG, "Add a error for test adjtime");
            struct timeval tv_now;
            gettimeofday(&tv_now, NULL); //Obtem a hora atual e armazena no endereço do ponteiro tv_now 
            int64_t cpu_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec; //Armazena o tempo em cpu_time, somando o tempo em segundos e microssegundos
            int64_t error_time = cpu_time + 500 * 1000L; // Adiciona 500ms para ajustar o método SMOOTH
            struct timeval tv_error = { .tv_sec = error_time / 1000000L, .tv_usec = error_time % 1000000L }; //Cria uma nova estrutura timeval para armazenar o valor do erro
            settimeofday(&tv_error, NULL);
        }

        ESP_LOGI(TAG, "Time was set, now just adjusting it. Use SMOOTH SYNC method.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }
#endif

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", "EST3", 1); // Função onde o fuso-horário é definido. EST3 - Fuso Recife.
    tzset(); //Inicializada a variável TZ 
    localtime_r(&now, &timeinfo); //Função que converte o valor do tempo em segundos desde a Epoch contido em "now" e armazena os dados detalhados de tempo em "timeinfo"
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo); //Converte os dados numéricos de tempo contidos em "timeinfo" em formato string para armazenar em "strftime_buf" da forma: 'Dia da semana' Mês Dia Hora:Minutos:Segundos Mês\0.
    ESP_LOGI(TAG, "The current date/time in Recife is: %s", strftime_buf); //Imprime a string
    ESP_LOGI(TAG, "Dia %d", timeinfo.tm_mday); //Imprime o dia
    ESP_LOGI(TAG, "Mês %d", timeinfo.tm_mon); 
    ESP_LOGI(TAG, "Ano %d", timeinfo.tm_year);
    printf("%.2d/%.2d/%.2d %.2d:%.2d:%.2d", timeinfo.tm_mday,timeinfo.tm_mon+1,timeinfo.tm_year+1900,timeinfo.tm_hour,timeinfo.tm_min,timeinfo.tm_sec); //Formatação do tempo e impressão como: dia/mês/ano hora:minuto:segundo

    if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
        struct timeval outdelta;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) { //Loop para ajuste da hora do sistema. Permanece no loop enquanto o ajuste não for finalizado.
            adjtime(NULL, &outdelta); //Corrige a hora para permitir a sincronização do relógio do sistema
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }


    // Start ADS loop
    //xTaskCreate(xADS1115_thred, "xADS1115_thred",2048, NULL, 3, NULL);


    //Inicialização das tarefas que serão executadas no core 0 e 1 do esp32
    //nvs_flash_init();
	//xTaskCreatePinnedToCore(&hello_task_core_0, "core1_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_0);
	//xTaskCreatePinnedToCore(&hello_task_core_1, "core0_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_1);
    xTaskCreatePinnedToCore(&hello_task_core_1, "core1_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_1);
	xTaskCreatePinnedToCore(&hello_task_core_0, "core0_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_0);

}


static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() ); //Verifica se há erro na inicialização da partição do armazenamento não-volátil da Flash
    ESP_ERROR_CHECK(esp_netif_init()); //Verifica se há erro na inicialização da pilha TCP/IP
    ESP_ERROR_CHECK( esp_event_loop_create_default() ); // Verifica se há erro na criação um loop de eventos padrão

    /**
     * NTP server address could be aquired via DHCP,
     * see LWIP_DHCP_GET_NTP_SRV menuconfig option
     */
#ifdef LWIP_DHCP_GET_NTP_SRV
    sntp_servermode_dhcp(1); //Configura o servidor SNTP
#endif

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect()); //Verifica se há erro na conexão da Ethernet ou WiFi. Função "example_connect" fica bloqueada até que a conexão seja estabelecida e IP obtido

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    ESP_ERROR_CHECK( example_disconnect() );
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL); //Define o modo de operação SNTP
    sntp_setservername(0, "pool.ntp.org"); //Define o nome do host SNTP. Os parâmetros são índice e nome do servidor
    sntp_set_time_sync_notification_cb(time_sync_notification_cb); //Define uma função que será chamada após a sincronização de horário
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH); //Define o modo de sincronização. "SNTP_SYNC_MODE_SMOOTH" Atualiza o tempo suavemente, reduzindo gradualmente o erro de tempo usando a função adjtime()
#endif
    sntp_init(); //Inicia o serviço SNTP.
}