#include "Fan.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"


void set_motor_speed(uint32_t duty_cycle) {
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty_cycle);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}
void reverse_motor_direction() {
    
}
void run_fan_for_minutes(uint32_t minutes) {
    uint32_t seconds = minutes * 60; 
    uint32_t start_time = esp_timer_get_time() / 1000000; 

    while ((esp_timer_get_time() / 1000000) - start_time < seconds) {
        set_motor_speed(DUTY_MEDIUM);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
