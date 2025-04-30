#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1 GPIO_NUM_26
#define LED2 GPIO_NUM_27

// Função que faz o LED1 piscar
void led1_vaiPiscar() {
    while (1) {
        gpio_set_level(LED1, 1);
        vTaskDelay(pdMS_TO_TICKS(200));  // Espera 200ms
        gpio_set_level(LED1, 0);
        vTaskDelay(pdMS_TO_TICKS(200));  // Espera 200ms
    }
}

// Função que faz o LED2 piscar
void led2_vaiPiscar() {
    while (1) {
        gpio_set_level(LED2, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1000ms
        gpio_set_level(LED2, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1000ms
    }
}

// Função principal
void app_main() {
    // Configura os pinos como saída
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    
    // Cria as tarefas para os LEDs piscarem
    xTaskCreate(led1_vaiPiscar, "led1", 1024, NULL, 1, NULL);
    xTaskCreate(led2_vaiPiscar, "led2", 1024, NULL, 1, NULL);
    
    // Código para piscar LED adicional (GPIO_NUM_2 e GPIO_NUM_14)
    while (1) {
        gpio_set_level(GPIO_NUM_26, 1);
        vTaskDelay(pdMS_TO_TICKS(200));  // Espera 200ms
        gpio_set_level(GPIO_NUM_26, 0);
        vTaskDelay(pdMS_TO_TICKS(200));  // Espera 200ms

        gpio_set_level(GPIO_NUM_27, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1000ms
        gpio_set_level(GPIO_NUM_27, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));  // Espera 1000ms
    }
}




