/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO12 12
#define BLINK_GPIO13 13
#define BLINK_GPIO14 14
#define BLINK_GPIO27 27


static uint8_t gpio12_state = 1;
static uint8_t gpio13_state = 1;
static uint8_t gpio14_state = 1;
static uint8_t gpio27_state = 1;
static uint64_t numero = 0;

#ifdef CONFIG_BLINK_LED_RMT
static led_strip_t *pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (gpio_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    } else {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(int BLINK_GPIO,int gpio_state)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, gpio_state);
}

static void configure_led(int BLINK_GPIO)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
   
}

#endif

int sort()
{
    int number;
    //printf("intervalo da rand: [0,%d]\n", RAND_MAX);
    srand( (unsigned)time(NULL) );
        
    number = rand() % 100;   // Gera um número entre 0 e 99
    //for(i=1 ; i <= 10 ; i++)
    printf("Numero: %d\n", number);
    return number;
}


void app_main(void)
{

    /* Configure the peripheral according to the LED type */
    configure_led(BLINK_GPIO12);  //FIO VERDE - Acionar RELÉ 2 (40W e 0)
    configure_led(BLINK_GPIO13);  //FIO AMARELO - Acionar RELÉ 1 (100W e 15W)
    configure_led(BLINK_GPIO14);  //FIO AZUL - Acionar RELÉ 3 (OFF)
    configure_led(BLINK_GPIO27);  //FIO BRANCO - Acionar RELÉ 4 (15W e 0)

    while (1) {
        ESP_LOGI(TAG, "Turning the LED %s!", gpio13_state == true ? "ON" : "OFF");
        //blink_led();
        /* Toggle the LED state */
        //s_led_state = !s_led_state;
        numero = sort();
        if (numero == 87) {  // lâmpada amarela apaga e lâmpada verde acende
            gpio12_state = 0;
            gpio14_state = 1;
            gpio27_state = 0;
            gpio13_state = 1;
            blink_led(BLINK_GPIO14,gpio14_state);
            blink_led(BLINK_GPIO13,gpio13_state);
            blink_led(BLINK_GPIO12,gpio12_state);
            blink_led(BLINK_GPIO27,gpio27_state);
            printf("100W\n");
            //printf("QUEDA DE ENERGIA DE 5s\n");
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            //gpio13_state = 1;
            //blink_led(BLINK_GPIO13,gpio13_state);
        }
        if (87 < numero && numero < 100 ) {  // lâmpada branca apaga e lâmpada azul acende
            gpio12_state = 1;
            gpio14_state = 1;
            gpio27_state = 0;
            gpio13_state = 1;
            blink_led(BLINK_GPIO14,gpio14_state);
            blink_led(BLINK_GPIO13,gpio13_state);
            blink_led(BLINK_GPIO12,gpio12_state);
            blink_led(BLINK_GPIO27,gpio27_state);
            printf("140W\n");
            
            if (88 == numero || numero == 89) {
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
            if (90 == numero || numero == 91) {
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
            if (92 == numero || numero == 93 || numero == 94) {
                vTaskDelay(8000 / portTICK_PERIOD_MS);
            }
            if (95 == numero || numero == 96 || numero == 97) {
                vTaskDelay(12000 / portTICK_PERIOD_MS);
            }
            if (98 == numero) {
                vTaskDelay(16000 / portTICK_PERIOD_MS);
            }
            if (99 == numero) {
                vTaskDelay(20000 / portTICK_PERIOD_MS);
            }


        }
        if (46 < numero && numero < 87) {  // Simulação de queda de energia
            gpio12_state = 1;
            gpio14_state = 1;
            gpio27_state = 1;
            gpio13_state = 0;
            blink_led(BLINK_GPIO14,gpio14_state);
            blink_led(BLINK_GPIO13,gpio13_state);
            blink_led(BLINK_GPIO12,gpio12_state);
            blink_led(BLINK_GPIO27,gpio27_state);
            printf("70W\n");
            if (47 == numero || (numero > 47 && numero < 77)) {
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
            if (77 == numero || (numero > 77 && numero < 80)) {
                vTaskDelay(8000 / portTICK_PERIOD_MS);
            }
            if (80 == numero || (numero > 80 && numero < 84)) {
                vTaskDelay(12000 / portTICK_PERIOD_MS);
            }
            if (84 == numero || numero == 85) {
                vTaskDelay(16000 / portTICK_PERIOD_MS);
            }
            if (86 == numero) {
                vTaskDelay(20000 / portTICK_PERIOD_MS);
            }

        }
        if (numero == 45 || numero == 46) {  // Pastilha de peltier
            gpio12_state = 0;
            gpio14_state = 1;
            gpio27_state = 1;
            gpio13_state = 0;
            blink_led(BLINK_GPIO14,gpio14_state);
            blink_led(BLINK_GPIO13,gpio13_state);
            blink_led(BLINK_GPIO12,gpio12_state);
            blink_led(BLINK_GPIO27,gpio27_state);
            printf("30W\n");
            vTaskDelay(2000 / portTICK_PERIOD_MS);

        }
        if (0 == numero || (numero > 0 && numero < 45)) {
            gpio12_state = 0;
            gpio14_state = 0;
            gpio27_state = 0;
            gpio13_state = 1;
            blink_led(BLINK_GPIO14,gpio14_state);
            blink_led(BLINK_GPIO13,gpio13_state);
            blink_led(BLINK_GPIO12,gpio12_state);
            blink_led(BLINK_GPIO27,gpio27_state);
            printf("OFF\n");

            if (0 == numero || (numero > 0 && numero < 25)) {
                vTaskDelay(4000 / portTICK_PERIOD_MS);
            }
            if (25 == numero || (numero > 25 && numero < 32)) {
                vTaskDelay(8000 / portTICK_PERIOD_MS);
            }
            if (32 == numero || (numero > 32 && numero < 40)) {
                vTaskDelay(12000 / portTICK_PERIOD_MS);
            }
            if (40 == numero || (numero > 40 && numero < 42)) {
                vTaskDelay(16000 / portTICK_PERIOD_MS);
            }
            if (42 == numero || (numero > 42 && numero < 45)) {
                vTaskDelay(20000 / portTICK_PERIOD_MS);
            }
        }
        //vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);

        //vTaskDelay(300);
    }
}
