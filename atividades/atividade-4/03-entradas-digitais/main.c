#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#define BTN_INC     GPIO_NUM_18  
#define BTN_TOGGLE  GPIO_NUM_19  

#define DEBOUNCE_TIME_US 300000  

const gpio_num_t leds[4] = { GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17 };

uint8_t counter = 0;
uint8_t increment = 1;

int64_t last_inc_time = 0;
int64_t last_toggle_time = 0;

void init_gpio() {
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
    }

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_TOGGLE),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    gpio_config(&btn_conf);
}

void update_leds(uint8_t value) {
    for (int i = 0; i < 4; i++) {
        gpio_set_level(leds[i], (value >> i) & 0x01);
    }
}

bool button_pressed(gpio_num_t pin, int64_t *last_time) {
    if (gpio_get_level(pin) == 0) {
        int64_t now = esp_timer_get_time();
        if (now - *last_time > DEBOUNCE_TIME_US) {
            *last_time = now;
            return true;
        }
    }
    return false;
}

void app_main(void) {
    init_gpio();
    update_leds(counter);

    while (1) {
        if (button_pressed(BTN_INC, &last_inc_time)) {
            counter = (counter + increment) % 16;
            update_leds(counter);
        }

        if (button_pressed(BTN_TOGGLE, &last_toggle_time)) {
            increment = (increment == 1) ? 2 : 1;
        }
    }
}
