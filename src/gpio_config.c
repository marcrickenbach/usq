/**********************************************************************************
 * Note: This is a hold over from previous project. This isn't compiled. Just here
 * for reference. 
 * 
 *
*/

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <assert.h>
#include "gpio_config.h"

K_THREAD_STACK_DEFINE(gpio_config_stack, GPIO_CONFIG_STACK_SIZE);

// Gate Outs
const struct gpio_dt_spec seq_gate_high = GPIO_DT_SPEC_GET(GPIO_PINS, gate_high_gpios); 
const struct gpio_dt_spec seq_gate_low  = GPIO_DT_SPEC_GET(GPIO_PINS, gate_low_gpios); 

// Button LEDS
const struct gpio_dt_spec play_led_high  = GPIO_DT_SPEC_GET(GPIO_PINS, led_17_gpios); 
const struct gpio_dt_spec reset_led_high = GPIO_DT_SPEC_GET(GPIO_PINS, led_18_gpios); 
const struct gpio_dt_spec play_led_low   = GPIO_DT_SPEC_GET(GPIO_PINS, led_19_gpios); 
const struct gpio_dt_spec reset_led_low  = GPIO_DT_SPEC_GET(GPIO_PINS, led_20_gpios); 
const struct gpio_dt_spec mode_led       = GPIO_DT_SPEC_GET(GPIO_PINS, led_21_gpios); 

// MUX Address Pins
const struct gpio_dt_spec adc_address_0 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_0_gpios); 
const struct gpio_dt_spec adc_address_1 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_1_gpios); 
const struct gpio_dt_spec adc_address_2 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_2_gpios); 

const struct gpio_dt_spec sw_address_0 = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_0_gpios); 
const struct gpio_dt_spec sw_address_1 = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_1_gpios); 
const struct gpio_dt_spec sw_address_2 = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_2_gpios); 

// MUX Bank Select Pins
const struct gpio_dt_spec adc_bank_0 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_bank_0_gpios); 
const struct gpio_dt_spec adc_bank_1 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_bank_1_gpios); 
const struct gpio_dt_spec adc_bank_2 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_bank_2_gpios); 
const struct gpio_dt_spec adc_bank_3 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_bank_3_gpios); 
const struct gpio_dt_spec adc_bank_4 = GPIO_DT_SPEC_GET(GPIO_PINS, adc_addr_bank_4_gpios); 

const struct gpio_dt_spec sw_bank_0 = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_bank_0_gpios); 
const struct gpio_dt_spec sw_bank_1 = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_bank_1_gpios); 
 


void gpio_config_thread (void * p1, void * p2, void * p3) {
    /* Note: there is no loop, this runs only once, then we exit and kill thread*/

    if  (   !device_is_ready(seq_gate_high.port)    ||
            !device_is_ready(seq_gate_low.port)     ||
            !device_is_ready(play_led_high.port)    ||
            !device_is_ready(reset_led_high.port)   ||
            !device_is_ready(play_led_low.port)     ||
            !device_is_ready(reset_led_low.port)    ||
            !device_is_ready(mode_led.port)         ||
            !device_is_ready(adc_address_0.port)    ||
            !device_is_ready(adc_address_1.port)    ||
            !device_is_ready(adc_address_2.port)    ||
            !device_is_ready(sw_address_0.port)     ||
            !device_is_ready(sw_address_1.port)     ||
            !device_is_ready(sw_address_2.port)     ||
            !device_is_ready(adc_bank_0.port)       ||
            !device_is_ready(adc_bank_1.port)       ||
            !device_is_ready(adc_bank_2.port)       ||
            !device_is_ready(adc_bank_3.port)       ||
            !device_is_ready(adc_bank_4.port)       ||
            !device_is_ready(sw_bank_0.port)        ||
            !device_is_ready(sw_bank_1.port)  
        ){
        
        printk("GPIO Ready: Failed\n");
        return; 
        
    } else {

            printk("GPIO Ready: OK\n");

    }
    
     

    if  (   gpio_pin_configure_dt(&seq_gate_high, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&seq_gate_low, GPIO_OUTPUT_INACTIVE)   ||
            gpio_pin_configure_dt(&play_led_high, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&reset_led_high, GPIO_OUTPUT_INACTIVE) ||
            gpio_pin_configure_dt(&play_led_low, GPIO_OUTPUT_INACTIVE)   ||
            gpio_pin_configure_dt(&reset_led_low, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&mode_led, GPIO_OUTPUT_INACTIVE)       ||
            gpio_pin_configure_dt(&adc_address_0, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&adc_address_1, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&adc_address_2, GPIO_OUTPUT_INACTIVE)  ||
            gpio_pin_configure_dt(&sw_address_0, GPIO_OUTPUT_INACTIVE)   ||
            gpio_pin_configure_dt(&sw_address_1, GPIO_OUTPUT_INACTIVE)   ||
            gpio_pin_configure_dt(&sw_address_2, GPIO_OUTPUT_INACTIVE)   ||
            gpio_pin_configure_dt(&adc_bank_0, GPIO_OUTPUT_INACTIVE)     ||
            gpio_pin_configure_dt(&adc_bank_1, GPIO_OUTPUT_INACTIVE)     ||
            gpio_pin_configure_dt(&adc_bank_2, GPIO_OUTPUT_INACTIVE)     ||
            gpio_pin_configure_dt(&adc_bank_3, GPIO_OUTPUT_INACTIVE)     ||
            // gpio_pin_configure_dt(&adc_bank_4, GPIO_OUTPUT_INACTIVE)     ||
            // FIXME: Pin above is in conflict with usart2 on the nucleo board. Uncomment when porting to custom board. 
            gpio_pin_configure_dt(&sw_bank_0, GPIO_OUTPUT_INACTIVE)      ||
            gpio_pin_configure_dt(&sw_bank_1, GPIO_OUTPUT_INACTIVE)     
            ){

        printk("GPIO Config: Failed\n");
        return; 

    } else {

        printk("GPIO Config: OK\n");

    }

}
