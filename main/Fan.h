#ifndef FAN_H
#define FAN_H

#include <stdint.h>

#define IN1_GPIO_NUM 26
#define IN2_GPIO_NUM 27
#define PWM_RESOLUTION LEDC_TIMER_13_BIT
#define PWM_FREQ 1000
#define LEDC_HIGH_SPEED_MODE LEDC_HIGH_SPEED_MODE

#define LEDC_CHANNEL_0 LEDC_CHANNEL_0
#define DUTY_LIGHT 3000
#define DUTY_MEDIUM 6000
#define DUTY_HIGH 8191

void set_motor_speed(uint32_t duty_cycle);
void reverse_motor_direction();
void run_fan_for_minutes(uint32_t minutes);

#endif /* FAN_H */