#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

static QueueHandle_t gpio_evt_queue = NULL;

typedef enum {
    BTN_INC_EVT,
    BTN_TOGGLE_EVT
} button_event_t;

static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    button_event_t evt;

    int64_t now = esp_timer_get_time();

    if (pin == BTN_INC && now - last_inc_time > DEBOUNCE_TIME_US) {
        last_inc_time = now;
        evt = BTN_INC_EVT;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    } else if (pin == BTN_TOGGLE && now - last_toggle_time > DEBOUNCE_TIME_US) {
        last_toggle_time = now;
        evt = BTN_TOGGLE_EVT;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    }
}

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
        .intr_type = GPIO_INTR_POSEDGE,  
    };
    gpio_config(&btn_conf);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(BTN_INC, gpio_isr_handler, (void*)BTN_INC);
    gpio_isr_handler_add(BTN_TOGGLE, gpio_isr_handler, (void*)BTN_TOGGLE);
}

void update_leds(uint8_t value) {
    for (int i = 0; i < 4; i++) {
        gpio_set_level(leds[i], (value >> i) & 0x01);
    }
}

void app_main(void) {
    init_gpio();
    update_leds(counter);

    gpio_evt_queue = xQueueCreate(10, sizeof(button_event_t));

    button_event_t evt;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
            switch (evt) {
                case BTN_INC_EVT:
                    counter = (counter + increment) % 16;
                    update_leds(counter);
                    break;

                case BTN_TOGGLE_EVT:
                    increment = (increment == 1) ? 2 : 1;
                    break;
            }
        }
    }
}
