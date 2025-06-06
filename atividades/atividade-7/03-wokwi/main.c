#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"

#define LED_PINS { GPIO_NUM_2, GPIO_NUM_4, GPIO_NUM_16, GPIO_NUM_17 }
#define BUZZER_PIN GPIO_NUM_13
#define BTN_INC GPIO_NUM_18
#define BTN_DEC GPIO_NUM_19
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_20
#define NTC_ADC_CHANNEL ADC_CHANNEL_0 

#define I2C_MASTER_NUM I2C_NUM_0
#define LCD_ADDR 0x27
#define LCD_ROWS 2
#define LCD_COLS 16

#define DEBOUNCE_US 300000
#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_13_BIT
#define TEMP_ALARME 25

static QueueHandle_t gpio_evt_queue = NULL;
volatile int temp_lida = 0;
int64_t last_btn_inc_time = 0;
int64_t last_btn_dec_time = 0;

gpio_num_t led_pins[4] = LED_PINS;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void lcd_cmd(uint8_t cmd) {
    uint8_t data[4] = {
        (cmd & 0xF0) | 0x0C,
        (cmd & 0xF0) | 0x08,
        ((cmd << 4) & 0xF0) | 0x0C,
        ((cmd << 4) & 0xF0) | 0x08
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data, 4, pdMS_TO_TICKS(100));
}

void lcd_data(uint8_t data_char) {
    uint8_t data[4] = {
        (data_char & 0xF0) | 0x0D,
        (data_char & 0xF0) | 0x09,
        ((data_char << 4) & 0xF0) | 0x0D,
        ((data_char << 4) & 0xF0) | 0x09
    };
    i2c_master_write_to_device(I2C_MASTER_NUM, LCD_ADDR, data, 4, pdMS_TO_TICKS(100));
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_cmd(0x33);
    lcd_cmd(0x32);
    lcd_cmd(0x28);
    lcd_cmd(0x0C);
    lcd_cmd(0x06);
    lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

void lcd_set_cursor(int col, int row) {
    lcd_cmd(0x80 | (col + (row ? 0x40 : 0x00)));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data((uint8_t)*str++);
    }
}

void update_lcd() {
    char linha1[17], linha2[17];
    snprintf(linha1, sizeof(linha1), "Temp: %2d C", temp_lida);
    snprintf(linha2, sizeof(linha2), "Alarme: %2d C", TEMP_ALARME);

    lcd_set_cursor(0, 0);
    lcd_print("                ");
    lcd_set_cursor(0, 0);
    lcd_print(linha1);

    lcd_set_cursor(0, 1);
    lcd_print("                ");
    lcd_set_cursor(0, 1);
    lcd_print(linha2);
}

void update_leds() {
    int diff = TEMP_ALARME - temp_lida;
    for (int i = 0; i < 4; i++) gpio_set_level(led_pins[i], 0);

    if (temp_lida >= TEMP_ALARME) {
        for (int i = 0; i < 4; i++) gpio_set_level(led_pins[i], (esp_timer_get_time() / 500000) % 2);
    } else if (diff <= 2) {
        for (int i = 0; i < 4; i++) gpio_set_level(led_pins[i], 1);
    } else if (diff <= 10) {
        for (int i = 0; i < 3; i++) gpio_set_level(led_pins[i], 1);
    } else if (diff <= 15) {
        for (int i = 0; i < 2; i++) gpio_set_level(led_pins[i], 1);
    } else if (diff <= 20) {
        gpio_set_level(led_pins[0], 1);
    }
}

void update_buzzer() {
    if (temp_lida >= TEMP_ALARME) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 4096);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

int read_temperature(adc_oneshot_unit_handle_t adc_handle) {
    static int fake_temp = 0;
    return fake_temp;
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
    gpio_num_t pin = (gpio_num_t)(uint32_t)arg;
    int64_t now = esp_timer_get_time();

    if (pin == BTN_INC && now - last_btn_inc_time > DEBOUNCE_US) {
        last_btn_inc_time = now;
        xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
    } else if (pin == BTN_DEC && now - last_btn_dec_time > DEBOUNCE_US) {
        last_btn_dec_time = now;
        xQueueSendFromISR(gpio_evt_queue, &pin, NULL);
    }
}

void app_main() {
    
    i2c_master_init();
    lcd_init();

    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(led_pins[i]);
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
    }

    ledc_timer_config_t pwm_timer = {
        .duty_resolution = PWM_RES,
        .freq_hz = PWM_FREQ,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&pwm_timer);

    ledc_channel_config_t pwm_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&pwm_channel);

    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&btn_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, gpio_isr_handler, (void *)BTN_INC);
    gpio_isr_handler_add(BTN_DEC, gpio_isr_handler, (void *)BTN_DEC);
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_num_t));

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11
    };
    adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &chan_cfg);

    while (1) {
        gpio_num_t btn;
        if (xQueueReceive(gpio_evt_queue, &btn, 0)) {
            if (btn == BTN_INC) temp_lida += 5;
            if (btn == BTN_DEC && temp_lida > 0) temp_lida -= 5;
        }

        update_lcd();
        update_leds();
        update_buzzer();

        vTaskDelay(pdMS_TO_TICKS(300));
    }
}
