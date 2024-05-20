#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "DHT.h"
#include "led_lib.h"
#include <servo_lib.h>
#include "driver/i2c.h"
#include "i2c-lcd.h"
#include "Fan.h"

#define LED_GPIO_DEFAULT GPIO_NUM_2
static const char *TAG = "commute";
char temp[10];
char hum [10];

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_NUM_0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

void DHT_task(void *pvParameter)
{
    setDHTgpio(GPIO_NUM_27);
    ESP_LOGI(TAG, "Starting DHT Task\n\n");

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    lcd_init();
    lcd_clear();

    while (1)
    {
        ESP_LOGI(TAG, "=== Reading DHT ===\n");
        int ret = readDHT();

        errorHandler(ret);

        ESP_LOGI(TAG, "Hum: %.1f Tmp: %.1f\n", getHumidity(), getTemperature());

        sprintf(temp, "temp: %.1f", getTemperature());
        lcd_put_cur(0, 0);
        lcd_send_string(temp);

        sprintf(hum,"hum: %.1f", getHumidity());
        lcd_put_cur(1,0);
        lcd_send_string(hum);

        // -- wait at least 10 sec before reading again ------------
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    //LCD and DHT commute../////////
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("*", ESP_LOG_INFO);

    esp_rom_gpio_pad_select_gpio(GPIO_NUM_27);

    xTaskCreate(&DHT_task, "DHT_task", 2048, NULL, 5, NULL);
    //###############################################################

    //LED commute...////////////////
    // Khởi tạo LED
    esp_err_t init_result = led_control_init(LED_GPIO_DEFAULT);
    if (init_result != ESP_OK) {
        printf("Không thể khởi tạo LED. Mã lỗi: %d\n", init_result);
        return;
    }

    // Bật LED
    led_control_turn_on(LED_GPIO_DEFAULT);
    printf("Den sang");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // Chờ 1 giây

    // Tắt LED
    led_control_turn_off(LED_GPIO_DEFAULT);
    //################################################################

    //Servo commute...///////////////
    int servo0_pin = 2;
    add_servo(servo0_pin,0);

    servo_setPosition(0,0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    servo_setPosition(90,0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    servo_setPosition(0,0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    delete_servo(0);
    //###############################################################

    // fans ...//////////////////////////
    // Cấu hình các chân GPIO để điều khiển động cơ qua driver L298N
    esp_rom_gpio_pad_select_gpio(IN1_GPIO_NUM);
    gpio_set_direction(IN1_GPIO_NUM, GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(IN2_GPIO_NUM);
    gpio_set_direction(IN2_GPIO_NUM, GPIO_MODE_OUTPUT);

    // Cấu hình PWM
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = IN1_GPIO_NUM,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_sel = LEDC_TIMER_0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
    printf("Quạt sẽ quay trong vòng 1 phút\n");
    run_fan_for_minutes(1);
    set_motor_speed(0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    while (1) {
        set_motor_speed(DUTY_LIGHT);
        printf("Speed: Light\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS); 

        set_motor_speed(DUTY_MEDIUM);
        printf("Speed: Medium\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS); 

        set_motor_speed(DUTY_HIGH);
        printf("Speed: High\n");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}