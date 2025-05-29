#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "int_i2c.h"

#define LED_PWM GPIO_NUM_13
const gpio_num_t leds[4] = { GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17 };
#define BTN_INC GPIO_NUM_18
#define BTN_DEC GPIO_NUM_19
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_20

#define DEBOUNCE_TIME_US 300000
#define MAX_COUNT 15

uint8_t counter = 0;
int64_t last_inc_time = 0;
int64_t last_dec_time = 0;

lcd_i2c_handle_t lcd = {
    .address = 0x27,
    .num = I2C_NUM_0,
    .backlight = 1,
    .size = DISPLAY_16X02
};

typedef enum {
    BTN_INC_EVT,
    BTN_DEC_EVT
} button_event_t;

static QueueHandle_t gpio_evt_queue = NULL;

void update_leds(uint8_t value) {
    for (int i = 0; i < 4; i++) {
        gpio_set_level(leds[i], (value >> i) & 0x01);
    }
}

void update_pwm(uint8_t value) {
    uint32_t duty = (value * 8191) / 15;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void update_lcd(uint8_t value) {
    char hex_str[5] = {0};
    char dec_str[5] = {0};

    snprintf(hex_str, sizeof(hex_str), "%X", value);
    snprintf(dec_str, sizeof(dec_str), "%d", value);

    lcd_i2c_write(&lcd, 0, CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(5));  

    lcd_i2c_cursor_set(&lcd, 0, 0);
    lcd_i2c_print(&lcd, "HEX: %-3s", hex_str); 

    lcd_i2c_cursor_set(&lcd, 0, 1);
    lcd_i2c_print(&lcd, "DEC: %-3s", dec_str);
}


static void IRAM_ATTR gpio_isr_handler(void* arg) {
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    button_event_t evt;
    int64_t now = esp_timer_get_time();

    if (pin == BTN_INC && now - last_inc_time > DEBOUNCE_TIME_US) {
        last_inc_time = now;
        evt = BTN_INC_EVT;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    } else if (pin == BTN_DEC && now - last_dec_time > DEBOUNCE_TIME_US) {
        last_dec_time = now;
        evt = BTN_DEC_EVT;
        xQueueSendFromISR(gpio_evt_queue, &evt, NULL);
    }
}

void init_gpio() {
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
    }

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE,
    };
    gpio_config(&btn_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, gpio_isr_handler, (void*)BTN_INC);
    gpio_isr_handler_add(BTN_DEC, gpio_isr_handler, (void*)BTN_DEC);
}

void init_pwm() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t channel = {
        .gpio_num = LED_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&channel);
}

void init_i2c_lcd() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    lcd_i2c_init(&lcd);
}

// --- APP MAIN ---
void app_main(void) {
    init_gpio();
    init_pwm();
    init_i2c_lcd();

    gpio_evt_queue = xQueueCreate(10, sizeof(button_event_t));

    update_leds(counter);
    update_pwm(counter);
    update_lcd(counter);

    button_event_t evt;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &evt, portMAX_DELAY)) {
            switch (evt) {
                case BTN_INC_EVT:
                    counter = (counter + 1) % 16;
                    break;
                case BTN_DEC_EVT:
                    counter = (counter == 0) ? MAX_COUNT : counter - 1;
                    break;
            }

            update_leds(counter);
            update_pwm(counter);
            update_lcd(counter);
        }
    }
}
