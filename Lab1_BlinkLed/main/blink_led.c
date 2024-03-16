#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 2 
#define DELAY_TIME_MS 1000 

void blink_task(void *pvParameter) {
    while(1) {
        gpio_set_level(LED_PIN, 1); /
        printf("LED ON\n"); 
        vTaskDelay(DELAY_TIME_MS / portTICK_PERIOD_MS); 

        gpio_set_level(LED_PIN, 0); 
        printf("LED OFF\n"); 
        vTaskDelay(DELAY_TIME_MS / portTICK_PERIOD_MS); 
    }
}

void app_main() {
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); 

    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);                                                                                                                                                                                                                                                                                                          
}
