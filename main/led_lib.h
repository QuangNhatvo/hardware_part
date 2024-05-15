#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "esp_err.h"
#include "driver/gpio.h"

// #define LED_GPIO_DEFAULT GPIO_NUM_2

/**
 * @brief Khởi tạo LED Control
 * 
 * @param gpio_num: số GPIO của LED
 * @return esp_err_t: ESP_OK nếu thành công, ESP_ERR_INVALID_ARG nếu tham số không hợp lệ
 */
esp_err_t led_control_init(int gpio_num) {
    if(gpio_num < 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_rom_gpio_pad_select_gpio(gpio_num);
    gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT);
    return ESP_OK;
}

/**
 * @brief Bật đèn LED
 * 
 * @param gpio_num: số GPIO của LED
 */
void led_control_turn_on(int gpio_num) {
    gpio_set_level(gpio_num, 1);
}

/**
 * @brief Tắt đèn LED
 * 
 * @param gpio_num: số GPIO của LED
 */
void led_control_turn_off(int gpio_num) {
    gpio_set_level(gpio_num, 0);
}

#endif /* LED_CONTROL_H */
