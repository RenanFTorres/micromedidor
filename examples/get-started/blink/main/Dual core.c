#include <freertos/FreeRTOS.h>
#include <freertos/task.h> 
 
void TaskRunningOnAppCore(void *arg) {
    while(1) {
 
        Serial.print(__func__);
        Serial.print(" : ");
        Serial.print(xTaskGetTickCount());
        Serial.print(" : ");
        Serial.print("This loop runs on APP_CPU which id is:");
        Serial.println(xPortGetCoreID());
        Serial.println();
 
        vTaskDelay(100);
    }
}
 
void TaskRunningOnProtocolCore(void *arg) {
    while(1) {
 
        Serial.print(__func__);
        Serial.print(" : ");
        Serial.print(xTaskGetTickCount());
        Serial.print(" : ");
        Serial.print("This loop runs on PRO_CPU which id is:");
        Serial.println(xPortGetCoreID());
        Serial.println();
 
        vTaskDelay(100);
    }
}
 
void setup(){
    Serial.begin(115200);
 
    xTaskCreatePinnedToCore(TaskRunningOnAppCore, 
                        "TaskOnApp", 
                        2048, 
                        NULL, 
                        4, 
                        NULL,
                         APP_CPU_NUM);
    
    xTaskCreatePinnedToCore(TaskRunningOnProtocolCore, 
                        "TaskOnPro", 
                        2048, 
                        NULL, 
                        8, 
                        NULL, 
                        PRO_CPU_NUM);
}
 
void loop(){
    Serial.print(__func__);
    Serial.print(" : ");
    Serial.print(xTaskGetTickCount());
    Serial.print(" : ");
    Serial.print("Arduino loop is running on core:");
    Serial.println(xPortGetCoreID());
    Serial.println();
 
    delay(500);
}