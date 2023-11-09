#ifndef GPIOCONFIG_H
#define GPIOCONFIG_H

#include <zephyr/kernel.h>

void gpio_config_thread (void * p1, void * p2, void * p3); 
extern struct A_Object gpio_config_task;
extern struct A_Object_Cfg gpio_config_task_cfg;

extern const struct gpio_dt_spec seq_gate_high;
extern const struct gpio_dt_spec seq_gate_low; 
extern const struct gpio_dt_spec play_led_high; 
extern const struct gpio_dt_spec reset_led_high; 
extern const struct gpio_dt_spec play_led_low; 
extern const struct gpio_dt_spec reset_led_low; 
extern const struct gpio_dt_spec mode_led; 
extern const struct gpio_dt_spec adc_address_0; 
extern const struct gpio_dt_spec adc_address_1; 
extern const struct gpio_dt_spec adc_address_2; 
extern const struct gpio_dt_spec sw_address_0; 
extern const struct gpio_dt_spec sw_address_1; 
extern const struct gpio_dt_spec sw_address_2; 
extern const struct gpio_dt_spec adc_bank_0; 
extern const struct gpio_dt_spec adc_bank_1; 
extern const struct gpio_dt_spec adc_bank_2; 
extern const struct gpio_dt_spec adc_bank_3; 
extern const struct gpio_dt_spec adc_bank_4; 
extern const struct gpio_dt_spec sw_bank_0; 
extern const struct gpio_dt_spec sw_bank_1; 

#endif