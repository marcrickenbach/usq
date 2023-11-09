#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <assert.h>
#include <zephyr/smf.h>
#include "trigger.h"
#include "main.h"
#include "gate.h"
#include "sequencers.h"
#include "thread_init.h"
#include "configurations.h"

/* Configure our Play/Pause Triggers and Mode Select Button */

const struct gpio_dt_spec pp_high  = GPIO_DT_SPEC_GET(GPIO_PINS, pp_trigger_high_gpios); 
const struct gpio_dt_spec rst_high = GPIO_DT_SPEC_GET(GPIO_PINS, rst_trigger_high_gpios); 
const struct gpio_dt_spec pp_low   = GPIO_DT_SPEC_GET(GPIO_PINS, pp_trigger_low_gpios); 
const struct gpio_dt_spec rst_low  = GPIO_DT_SPEC_GET(GPIO_PINS, rst_trigger_low_gpios);
const struct gpio_dt_spec mode_sel = GPIO_DT_SPEC_GET(GPIO_PINS, mode_global_gpios);

struct gpio_callback pp_trig_high_cb;
struct gpio_callback rst_trig_high_cb;
struct gpio_callback pp_trig_low_cb;
struct gpio_callback rst_trig_low_cb;
struct gpio_callback mode_sel_cb; 


/* Play/Pause Trigger Callback Function Definitions */ 

void pp_trigger_high(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
       k_event_post(&seq_high_sm.smf_event, EVENT_RUN);
}

void pp_trigger_low(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
       k_event_post(&seq_low_sm.smf_event, EVENT_RUN);
}


/* Reset Trigger Callback Function Definitions */ 

void rst_trigger_high(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
       // Post Message to reset high channel to 0
       // TODO: regarding these reset interrupts. Since the only work to be done here is to change the current step back to 0, does it make sense to queue a work or message item, or to just handle it in the interrupt? In other words, would it take longer to queue than to just clear a val to 0? 
}

void rst_trigger_low(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
       // Post Message to reset low channel to 0
       // TODO: regarding these reset interrupts. Since the only work to be done here is to change the current step back to 0, does it make sense to queue a work or message item, or to just handle it in the interrupt? In other words, would it take longer to queue than to just clear a val to 0? 
}

void mode_interrupt(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
       // Toggle MODE Flag
       // FIXME: In a separate Mode thread we can have a while loop that continues only so long as this flag is high (to configure eoc triggers etc). We'll have to configure this interrupt to also fire on release (FALLING EDGE) so as to toggle our flag. 
}


/* Triggers are initialized in main */
/* TODO: Add MUX'd button pin */
int triggers_init (void) {
       if (   !gpio_is_ready_dt(&pp_high)   ||
              !gpio_is_ready_dt(&rst_high)  ||
              !gpio_is_ready_dt(&pp_low)    ||
              !gpio_is_ready_dt(&rst_low)   ||
              !gpio_is_ready_dt(&mode_sel)
       ) { 
              printk("Error: triggers are not configured\n");
              return 1;
       } else {
              printk("Triggers activated: OK\n"); 
       }
       
       gpio_pin_configure_dt(&pp_high, GPIO_INPUT);
       gpio_pin_configure_dt(&rst_high, GPIO_INPUT);
       gpio_pin_configure_dt(&pp_low, GPIO_INPUT);
       gpio_pin_configure_dt(&rst_low, GPIO_INPUT);
       gpio_pin_configure_dt(&mode_sel, GPIO_INPUT);
       
       gpio_pin_interrupt_configure_dt(&pp_high, GPIO_INT_EDGE_TO_ACTIVE);
       gpio_pin_interrupt_configure_dt(&rst_high, GPIO_INT_EDGE_TO_ACTIVE);
       gpio_pin_interrupt_configure_dt(&pp_low, GPIO_INT_EDGE_TO_ACTIVE);
       gpio_pin_interrupt_configure_dt(&rst_high, GPIO_INT_EDGE_TO_ACTIVE);
       gpio_pin_interrupt_configure_dt(&mode_sel, GPIO_INT_EDGE_TO_ACTIVE);

       gpio_init_callback(&pp_trig_high_cb, pp_trigger_high, BIT(pp_high.pin));
       gpio_init_callback(&rst_trig_high_cb, rst_trigger_high, BIT(rst_high.pin));
       gpio_init_callback(&pp_trig_low_cb, pp_trigger_low, BIT(pp_low.pin));
       gpio_init_callback(&rst_trig_low_cb, rst_trigger_low, BIT(rst_low.pin));
       gpio_init_callback(&mode_sel_cb, mode_interrupt, BIT(mode_sel.pin));

       gpio_add_callback(pp_high.port, &pp_trig_high_cb);
       gpio_add_callback(rst_high.port, &pp_trig_high_cb);
       gpio_add_callback(pp_low.port, &pp_trig_high_cb);
       gpio_add_callback(rst_low.port, &pp_trig_high_cb);
       gpio_add_callback(mode_sel.port, &pp_trig_high_cb);

       return 0; 
}
