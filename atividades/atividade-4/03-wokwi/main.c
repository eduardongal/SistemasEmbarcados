#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

const gpio_num_t leds[4] = { GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17 };

#define BTN_TOGGLE GPIO_NUM_18
#define BTN_NEXT   GPIO_NUM_19

bool led_states[4] = { false, false, false, false };
int current_led = 0;

#define DEBOUNCE_TIME_MS 200

void init_gpio() {
    
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);
    }

    gpio_config_t btn_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BTN_TOGGLE) | (1ULL << BTN_NEXT),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    gpio_config(&btn_config);
}

void app_main(void) {
    init_gpio();

    int last_toggle = 1;
    int last_next = 1;

    while (1) {
        int toggle_val = gpio_get_level(BTN_TOGGLE);
        int next_val = gpio_get_level(BTN_NEXT);

        if (toggle_val == 0 && last_toggle == 1) {
            led_states[current_led] = !led_states[current_led];
            gpio_set_level(leds[current_led], led_states[current_led]);
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
        }

        if (next_val == 0 && last_next == 1) {
            current_led = (current_led + 1) % 4;
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
        }

        last_toggle = toggle_val;
        last_next = next_val;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}