#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "driver/i2c.h"

#define BUZZER_PIN GPIO_NUM_13
#define BTN_INC GPIO_NUM_18
#define BTN_DEC GPIO_NUM_19
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_20
#define NTC_ADC_CHANNEL ADC_CHANNEL_0

#define SEG_A GPIO_NUM_15
#define SEG_B GPIO_NUM_16
#define SEG_C GPIO_NUM_2
#define SEG_D GPIO_NUM_8
#define SEG_E GPIO_NUM_3
#define SEG_F GPIO_NUM_7
#define SEG_G GPIO_NUM_6

#define SPI_MISO GPIO_NUM_10
#define SPI_MOSI GPIO_NUM_11
#define SPI_CLK  GPIO_NUM_12
#define SPI_CS   GPIO_NUM_14

#define LCD_ADDR 0x27
#define I2C_PORT I2C_NUM_0
#define LCD_COLS 16
#define LCD_ROWS 2

#define DEBOUNCE_US 300000
#define PWM_FREQ 5000
#define PWM_RES LEDC_TIMER_13_BIT

#define NTC_SERIES_RESISTOR 10000.0
#define NTC_BETA 3950.0
#define NTC_ROOM_TEMP 298.15
#define NTC_ROOM_RESISTANCE 10000.0

#define MOUNT_POINT "/sdcard"

static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t xMutex;
adc_oneshot_unit_handle_t adc_handle;

float temp_ntc = 0;
int temp_alarme = 25;
bool alarme_ativo = false;
bool sd_montado = false;
int64_t last_btn_inc_time = 0;
int64_t last_btn_dec_time = 0;

int contador_inc = 0;

void i2c_master_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void lcd_cmd(uint8_t cmd) {
    uint8_t data[4] = {
        (cmd & 0xF0) | 0x0C, (cmd & 0xF0) | 0x08,
        ((cmd << 4) & 0xF0) | 0x0C, ((cmd << 4) & 0xF0) | 0x08
    };
    i2c_master_write_to_device(I2C_PORT, LCD_ADDR, data, 4, pdMS_TO_TICKS(100));
}

void lcd_data(uint8_t data_char) {
    uint8_t data[4] = {
        (data_char & 0xF0) | 0x0D, (data_char & 0xF0) | 0x09,
        ((data_char << 4) & 0xF0) | 0x0D, ((data_char << 4) & 0xF0) | 0x09
    };
    i2c_master_write_to_device(I2C_PORT, LCD_ADDR, data, 4, pdMS_TO_TICKS(100));
}

void lcd_set_cursor(int col, int row) {
    lcd_cmd(0x80 | (col + (row ? 0x40 : 0x00)));
}

void lcd_print(const char *str) {
    while (*str) lcd_data((uint8_t)*str++);
}

void lcd_init() {
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_cmd(0x33); lcd_cmd(0x32);
    lcd_cmd(0x28); lcd_cmd(0x0C);
    lcd_cmd(0x06); lcd_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(5));
}

float read_temperature() {
    int raw;
    adc_oneshot_read(adc_handle, NTC_ADC_CHANNEL, &raw);
    float voltage = raw * 3.3 / 4095.0;
    float resistance = (3.3 - voltage) * NTC_SERIES_RESISTOR / voltage;
    float tempK = 1.0 / (1.0 / NTC_ROOM_TEMP + log(resistance / NTC_ROOM_RESISTANCE) / NTC_BETA);
    return tempK - 273.15;
}

void buzzer_update() {
    int duty = (temp_ntc >= temp_alarme) ? 4096 : 0;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    alarme_ativo = (duty > 0);
}

void update_7seg(int numero) {
    numero %= 10;
    const uint8_t digitos[10][7] = {
        {1,1,1,1,1,1,0}, {0,1,1,0,0,0,0}, {1,1,0,1,1,0,1},
        {1,1,1,1,0,0,1}, {0,1,1,0,0,1,1}, {1,0,1,1,0,1,1},
        {1,0,1,1,1,1,1}, {1,1,1,0,0,0,0}, {1,1,1,1,1,1,1},
        {1,1,1,1,0,1,1}
    };
    gpio_set_level(SEG_A, digitos[numero][0]);
    gpio_set_level(SEG_B, digitos[numero][1]);
    gpio_set_level(SEG_C, digitos[numero][2]);
    gpio_set_level(SEG_D, digitos[numero][3]);
    gpio_set_level(SEG_E, digitos[numero][4]);
    gpio_set_level(SEG_F, digitos[numero][5]);
    gpio_set_level(SEG_G, digitos[numero][6]);
}

void init_sdcard() {
    sdmmc_card_t *card;
    const sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SPI_CS;
    slot_config.host_id = host.slot;
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SD card montado");
        sd_montado = true;
    } else {
        ESP_LOGE(TAG, "Erro ao montar SD: %s", esp_err_to_name(ret));
        sd_montado = false;
    }
}

void salvar_no_sd() {
    if (!sd_montado) return;
    FILE *f = fopen("/sdcard/leituras.txt", "a");
    if (f) {
        fprintf(f, "Temperatura: %.2f C\n", temp_ntc);
        fclose(f);
    }
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

void task_read_temp(void *pv) {
    while (1) {
        float t = read_temperature();
        xSemaphoreTake(xMutex, portMAX_DELAY);
        temp_ntc = t;
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_buzzer(void *pv) {
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        buzzer_update();
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void task_lcd(void *pv) {
    char linha1[17], linha2[17];
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        snprintf(linha1, sizeof(linha1), "Temp: %.1f C", temp_ntc);
        snprintf(linha2, sizeof(linha2), "Alarme: %2d C", temp_alarme);
        xSemaphoreGive(xMutex);

        lcd_set_cursor(0, 0); lcd_print("                ");
        lcd_set_cursor(0, 0); lcd_print(linha1);
        lcd_set_cursor(0, 1); lcd_print("                ");
        lcd_set_cursor(0, 1); lcd_print(linha2);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void task_sd(void *pv) {
    while (1) {
        salvar_no_sd();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void task_7seg(void *pv) {
    while (1) {
        xSemaphoreTake(xMutex, portMAX_DELAY);
        update_7seg(contador_inc); // Agora mostra o número de apertos do botão INC
        xSemaphoreGive(xMutex);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void task_buttons(void *pv) {
    gpio_num_t pin;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &pin, portMAX_DELAY)) {
            xSemaphoreTake(xMutex, portMAX_DELAY);
            if (pin == BTN_INC) {
                contador_inc++; // incrementa contador
                temp_alarme += 5;
            }
            if (pin == BTN_DEC && temp_alarme >= 5) {
                temp_alarme -= 5;
            }
            xSemaphoreGive(xMutex);
        }
    }
}

void app_main(void) {
    xMutex = xSemaphoreCreateMutex();
    gpio_evt_queue = xQueueCreate(10, sizeof(gpio_num_t));

    i2c_master_init(); lcd_init();

    gpio_reset_pin(SEG_A); gpio_set_direction(SEG_A, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_B); gpio_set_direction(SEG_B, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_C); gpio_set_direction(SEG_C, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_D); gpio_set_direction(SEG_D, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_E); gpio_set_direction(SEG_E, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_F); gpio_set_direction(SEG_F, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_G); gpio_set_direction(SEG_G, GPIO_MODE_OUTPUT);

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

    gpio_config_t btn_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&btn_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, gpio_isr_handler, (void *)BTN_INC);
    gpio_isr_handler_add(BTN_DEC, gpio_isr_handler, (void *)BTN_DEC);

    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);
    adc_oneshot_chan_cfg_t chan_cfg = { .bitwidth = ADC_BITWIDTH_DEFAULT, .atten = ADC_ATTEN_DB_11 };
    adc_oneshot_config_channel(adc_handle, NTC_ADC_CHANNEL, &chan_cfg);

    init_sdcard();

    xTaskCreate(task_read_temp, "read_temp", 2048, NULL, 5, NULL);
    xTaskCreate(task_buzzer, "buzzer", 2048, NULL, 5, NULL);
    xTaskCreate(task_lcd, "lcd", 2048, NULL, 5, NULL);
    xTaskCreate(task_sd, "sd", 4096, NULL, 4, NULL);
    xTaskCreate(task_7seg, "7seg", 2048, NULL, 4, NULL);
    xTaskCreate(task_buttons, "buttons", 2048, NULL, 6, NULL);
}
