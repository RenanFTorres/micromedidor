
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"

#define core_0 0
#define core_1 1

TaskHandle_t Task1;
TaskHandle_t Task2;

void hello_task_core_1(void *pvParameter)
{
	while(1){
	printf("Hello world from CORE %d!\n", xPortGetCoreID() );
    vTaskDelay(872 / portTICK_PERIOD_MS);
    fflush(stdout);
	} 
}

void hello_task_core_0(void *pvParameter)
{
	while(1) {
	printf("Hello world from CoRe %d!\n", xPortGetCoreID() );
    vTaskDelay(1323 / portTICK_PERIOD_MS);
    fflush(stdout);
	}
}

void app_main()
{
    //nvs_flash_init();
	xTaskCreatePinnedToCore(&hello_task_core_0, "core1_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_0);
	xTaskCreatePinnedToCore(&hello_task_core_1, "core0_task", 1024*4, NULL, configMAX_PRIORITIES - 1, NULL, core_1);
}