/* *****************************************************************************
 * @brief Sequencer implementation.
 */

/* *****************************************************************************
 * TODO
 */

/* *****************************************************************************
 * Includes
 */

#include "sequencer.h"

#include <zephyr/kernel.h>
#include <zephyr/smf.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <assert.h>

#include "sequencer/private/sm_evt.h"
#include "sequencer/private/module_data.h"

#include "pot.h"
#include "pot/evt.h"

#include "dac.h"
#include "dac/evt.h"

#include "led_driver.h"
#include "led_driver/evt.h"

#include "uart.h"
#include "uart/evt.h"

/* *****************************************************************************
 * Constants, Defines, and Macros
 */

#define SUCCESS             (0)
#define FAIL                (-1)

#define OVERRIDE            true
#define NO_OVERRIDE         false

#define TIMER_X             DT_INST(0, st_stm32_counter) 
#define TIMER_Y             DT_INST(1, st_stm32_counter)

// Gate Outs
#define GATE_PINS           DT_PATH(zephyr_user)
#define NUMBER_OF_GATES     2

// Interrupt Pins for Transport
#define TRANSPORT_PINS      DT_PATH(zephyr_user)

// Delay Coefficient for Testing
#define TEST_DELAY_COEFFICIENT  100
#define TEST_DELAY_TIME         120

#define GPIO_PINS           DT_PATH(zephyr_user)

#define FALLING_EDGE        0
#define RISING_EDGE         1

#define GATE_HIGH           1
#define GATE_LOW            0

/* LED Brightness */
#define FULL_BRIGHTNESS     0xFFF
#define HALF_BRIGHTNESS     0x7FF
#define QUARTER_BRIGHTNESS  0x3FF
#define EIGHTH_BRIGHTNESS   0x1FF
#define DIM_ARMED_STEP      0x1A

#define CHANNEL_STATE_ID  1

// Control Button Debouncing
#define DEBOUNCE_TIME_MS 50

/* *****************************************************************************
 * Debugging
 */

#if CONFIG_FKMG_SEQ_NO_OPTIMIZATIONS
#pragma GCC push_options
#pragma GCC optimize ("Og")
#warning "sequencer.c unoptimized!"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sequencer);


/* *****************************************************************************
 * Structs
 */

static struct sequencer_module_data sequencer_md = {0};
/* Convenience accessor to keep name short: md - module data. */
#define md sequencer_md

// Variable for counter configuration. This is just in case we change our counter frequency down the line, we can always work with a correct value. 
uint32_t counter_freq; 

/* *****************************************************************************
 * Device Structs
 */

const struct gpio_dt_spec gates[NUMBER_OF_GATES] = {
    GPIO_DT_SPEC_GET(GATE_PINS, gate_high_gpios), // Gate 0
    GPIO_DT_SPEC_GET(GATE_PINS, gate_low_gpios)   // Gate 1
};

const struct gpio_dt_spec playPauseA    = GPIO_DT_SPEC_GET(GPIO_PINS, pp_trigger_high_gpios); 
const struct gpio_dt_spec playPauseB    = GPIO_DT_SPEC_GET(TRANSPORT_PINS, pp_trigger_low_gpios); 
const struct gpio_dt_spec resetA        = GPIO_DT_SPEC_GET(TRANSPORT_PINS, rst_trigger_high_gpios);
const struct gpio_dt_spec resetB        = GPIO_DT_SPEC_GET(TRANSPORT_PINS, rst_trigger_low_gpios); 

const struct gpio_dt_spec play_led_high  = GPIO_DT_SPEC_GET(GPIO_PINS, led_17_gpios);
const struct gpio_dt_spec reset_led_high  = GPIO_DT_SPEC_GET(GPIO_PINS, led_18_gpios);
const struct gpio_dt_spec play_led_low  = GPIO_DT_SPEC_GET(GPIO_PINS, led_19_gpios);
const struct gpio_dt_spec reset_led_low  = GPIO_DT_SPEC_GET(GPIO_PINS, led_20_gpios);

const struct device *gpio_dev; 



/* *****************************************************************************
 * Private
 */

/* *****
 * Utils
 * *****/

static bool instance_contains_state_machine_ctx(
        struct Sequencer_Instance * p_inst,
        struct smf_ctx      * p_sm_ctx)
{
    return(&p_inst->sm == p_sm_ctx);
}

/* Find the instance that contains the state machine context. */
static struct Sequencer_Instance * sm_ctx_to_instance(struct smf_ctx * p_sm_ctx)
{
    /* Iterate thru the instances. */
    struct Sequencer_Instance * p_inst = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&md.list.instances, p_inst, node.instance){
        if(instance_contains_state_machine_ctx(p_inst, p_sm_ctx)) return p_inst;
    }
    return(NULL);
}


static enum Sequencer_Step_Id next_step(struct Sequencer_Instance * p_inst, enum Sequencer_Step_Id id)
{
    uint8_t step = p_inst->seq.step[id]; 
    return((step + 1) % p_inst->seq.maxStep[id]);
}

static void reset_timer(struct Sequencer_Instance * p_inst, 
                        enum Sequencer_Id id, 
                        int time_in_ms) {

    // Cancel the previous alarm
    int err = counter_cancel_channel_alarm(p_inst->timer.t[id], 0);
    if (err != 0) {
        LOG_ERR("Error: Failed to cancel Timer alarm");
        return;  // Handle error appropriately
    }

    // Calculate ticks for the new alarm time
    uint32_t ticks = (time_in_ms * counter_freq) / 1000;

    // Set up the new alarm
    struct counter_alarm_cfg timer_cfg = {
        .flags = 0,
        .ticks = ticks,
        .callback = p_inst->tim_cb[id],
        .user_data = p_inst
    };

    // Set the new alarm
    if (counter_set_channel_alarm(p_inst->timer.t[id], 0, &timer_cfg)) {
        LOG_ERR("Error: Failed to set new Timer alarm");
    }
}


/* 
 * Most work here only gets done on a rising edge, i.e. the on part of the gate. 
 * The on-time is calculated as half the time delay between when this rising edge hits and the next gate. 
 * The delay_buffer variable in the instance is to keep track of this delay so we only have to calculate it once,
 * unless one of an active channel's pots sends a pot_change event, in which case we will recalculate the new delay time. 
*/ 

static uint16_t calculate_gate_timer_delay(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, bool edge) 
{
    uint16_t delay_val; 

    if (edge) {
        delay_val = p_inst->seq.time[next_step(p_inst, id)]; 
        delay_val = delay_val >> 1; 
        // p_inst->seq.edge[id] = true; 
        p_inst->seq.delay_buffer[id] = delay_val; 

        return delay_val; 
    } 
    
    return p_inst->seq.delay_buffer[id]; 
}

/* **************
 * Listener Utils
 * **************/

#if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
static enum Sequencer_Err_Id check_listener_cfg_param_for_errors(
        struct Sequencer_Listener_Cfg * p_cfg )
{
    if(!p_cfg
    || !p_cfg->p_iface
    || !p_cfg->p_lsnr) return(k_Seq_Err_Id_Configuration_Invalid);

    /* Is signal valid? */
    if(p_cfg->sig > k_Seq_Sig_Max)
        return(k_Seq_Err_Id_Configuration_Invalid);

    /* Is callback valid? */
    if(!p_cfg->cb)
        return(k_Seq_Err_Id_Configuration_Invalid);

    return(k_Seq_Err_Id_None);
}
#endif

static void clear_listener(struct Sequencer_Listener * p_lsnr)
{
    memset(p_lsnr, 0, sizeof(*p_lsnr));
}

static void config_listener(
        struct Sequencer_Listener     * p_lsnr,
        struct Sequencer_Listener_Cfg * p_cfg)
{
    /* Set listner's instance it is listening to. */
    p_lsnr->p_inst = p_cfg->p_inst;

    /* Set listner's callback. */ 
    p_lsnr->cb = p_cfg->cb;
}

static void init_listener(struct Sequencer_Listener * p_lsnr)
{
    clear_listener(p_lsnr);
}

/* **************
 * Instance Utils
 * **************/

#if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
static enum Seq_Err_Id check_instance_cfg_param_for_errors(
        struct Seq_Instance_Cfg * p_cfg)
{
    if(!p_cfg
    || !p_cfg->p_inst
    || !p_cfg->task.sm.p_thread
    || !p_cfg->task.sm.p_stack
    || !p_cfg->msgq.p_sm_evts) return(k_Seq_Err_Id_Configuration_Invalid);

    if(p_cfg->task.sm.stack_sz == 0) return(k_Seq_Err_Id_Configuration_Invalid);

    return(k_Seq_Err_Id_None);
}
#endif

static void add_instance_to_instances(
        struct Sequencer_Instance  * p_inst)
{
    sys_slist_append(&md.list.instances, &p_inst->node.instance);
}

static void config_instance_queues(
        struct Sequencer_Instance     * p_inst,
        struct Sequencer_Instance_Cfg * p_cfg)
{
    p_inst->msgq.p_sm_evts = p_cfg->msgq.p_sm_evts;
}

/* Forward reference */

static void timer_x_callback (const struct device *timer_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data); 
static void timer_y_callback (const struct device *timer_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data); 

/* Set up hardware timers for main sequencers. */
static void init_hardware_timers(struct Sequencer_Instance * p_inst,
                                 struct Sequencer_Instance_Cfg * p_cfg)
{
    p_inst->timer.t[0] = DEVICE_DT_GET(TIMER_X);
    p_inst->timer.t[1] = DEVICE_DT_GET(TIMER_Y);

    counter_alarm_callback_t cb_cfg[2] = {
        timer_x_callback,
        timer_y_callback
    }; 

    for (int i = 0; i < 2; i++) {
        p_inst->tim_cb[i] = cb_cfg[i]; 
    }

    if (!device_is_ready(p_inst->timer.t[0]) ||
        !device_is_ready(p_inst->timer.t[1])) 
    {
        LOG_ERR("Timers Config: FAILED.\n");
        return;
    }
    
    counter_freq = counter_get_frequency(p_inst->timer.t[0]);

    uint32_t ticks = (TEST_DELAY_TIME * counter_freq) / 1000;

    struct counter_alarm_cfg x_cfg = {
        .flags = 0,
        .ticks = ticks,
        .callback = p_inst->tim_cb[0],
        .user_data = p_inst
    }; 

    struct counter_alarm_cfg y_cfg = {
        .flags = 0,
        .ticks = ticks,
        .callback = p_inst->tim_cb[1],
        .user_data = p_inst
    }; 
    
    if (counter_set_channel_alarm(p_inst->timer.t[0], 0, &x_cfg) != 0) {
        LOG_ERR("Error: Failed to set Timer X"); 
    };

    if (counter_set_channel_alarm(p_inst->timer.t[1], 0, &y_cfg) != 0) {
        LOG_ERR("Error: Failed to set Timer Y"); 
    };

    p_inst->seq.edge[0] = 0; 
    p_inst->seq.edge[1] = 0; 

    // counter_start(p_inst->timer.t[0]);
    // counter_start(p_inst->timer.t[1]);

    return; 

}


static void on_transportA(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_transportB(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_resetA(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_resetB(const struct device *dev, struct gpio_callback *cb, uint32_t pins); 



static void transport_gpio_pin_config(void) {
    int ret = gpio_pin_configure(playPauseA.port, playPauseA.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure playPauseA pin");
    }

    ret = gpio_pin_configure(playPauseB.port, playPauseB.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure playPauseB pin");
    }

    ret = gpio_pin_configure(resetA.port, resetA.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure resetA pin");
    }

    ret = gpio_pin_configure(resetB.port, resetB.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure resetB pin");
    }

    if  (   !device_is_ready(play_led_high.port)
        ||  !device_is_ready(play_led_low.port)
        ||  !device_is_ready(reset_led_high.port)
        ||  !device_is_ready(reset_led_low.port)
        ){
        LOG_ERR("Control LEDs Setup: Failed");
        return; 
    } 

    ret = gpio_pin_configure_dt(&play_led_high, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&play_led_low, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&reset_led_high, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&reset_led_low, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }

}


static void transport_gpio_interrupt_config(void) {
    int ret = gpio_pin_interrupt_configure(playPauseA.port, playPauseA.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on playPauseA pin");
    }

    ret = gpio_pin_interrupt_configure(playPauseB.port, playPauseB.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on playPauseB pin");
    }

    ret = gpio_pin_interrupt_configure(resetA.port, resetA.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on resetA pin");
    }

    ret = gpio_pin_interrupt_configure(resetB.port, resetB.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on resetB pin");
    }


}


static void transport_gpio_callback_config(struct Sequencer_Instance * p_inst) {
    
    gpio_init_callback(&p_inst->transport_gpio_1_cb, on_transportA, BIT(playPauseA.pin));
    gpio_init_callback(&p_inst->transport_gpio_2_cb, on_transportB, BIT(playPauseB.pin));
    gpio_init_callback(&p_inst->transport_gpio_1_rst_cb, on_resetA, BIT(resetA.pin));
    gpio_init_callback(&p_inst->transport_gpio_2_rst_cb, on_resetB, BIT(resetB.pin));



    int ret = gpio_add_callback(playPauseA.port, &p_inst->transport_gpio_1_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on playPauseA pin");
    }

    ret = gpio_add_callback(playPauseB.port, &p_inst->transport_gpio_2_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on playPauseB pin");
    }

    ret = gpio_add_callback(resetA.port, &p_inst->transport_gpio_1_rst_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on resetA pin");
    }

    ret = gpio_add_callback(resetB.port, &p_inst->transport_gpio_2_rst_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on resetB pin");
    }



}


static void transport_gpio_init(struct Sequencer_Instance * p_inst) {
    
    int ret = 0; 

    if (!device_is_ready(playPauseA.port) ||
        !device_is_ready(playPauseB.port) ||
        !device_is_ready(resetA.port) ||
        !device_is_ready(resetB.port)) 
    {
        ret = 1;
    }

    assert(ret == 0);
    transport_gpio_pin_config();
    transport_gpio_interrupt_config();
    transport_gpio_callback_config(p_inst);

}

static void init_gate_gpios(struct Sequencer_Instance * p_inst) 
{
    for (int i = 0; i < NUMBER_OF_GATES; i++) {
        
        if (!device_is_ready(gates[i].port)) {
            LOG_ERR("GATE %d Init : FAILED.\n", i);
        } else {
        
        if (gpio_pin_configure_dt(&gates[i], GPIO_OUTPUT_INACTIVE)) {
                LOG_ERR("GATE %d Config : FAILED.\n", i);
            }
        }
    }
}

/* INITIAL VALUE CONFIGURATIONS */
/* FIXME: Testing Only. Replace with an initial ADC read routine?*/
static void config_initial_voltages(struct Sequencer_Instance * p_inst)
{
    for (int i = 0; i < ARRAY_SIZE(p_inst->seq.voltage); ++i)
    {
        p_inst->seq.voltage[i] = 0 + (i * 600);
    }
}

/* FIXME: Testing only. Replace with an initial ADC read routine? */
static void config_initial_time_delays(struct Sequencer_Instance * p_inst)
{
    for (int i = 0; i < ARRAY_SIZE(p_inst->seq.time); ++i)
    {
        p_inst->seq.time[i] = TEST_DELAY_TIME; ;//1 + (i * TEST_DELAY_COEFFICIENT);
    }
}


static void config_initial_sequencer_values(struct Sequencer_Instance * p_inst) {
    
    config_initial_voltages(p_inst); 
    config_initial_time_delays(p_inst);

    p_inst->seq.maxStep[0] = 8; 
    p_inst->seq.maxStep[1] = 8; 
    p_inst->seq.offset = 8; 

}


static void config_instance_deferred(
        struct Sequencer_Instance     * p_inst,
        struct Sequencer_Instance_Cfg * p_cfg)
{

    /* FIXME: These are for testing w/o hardware */
    config_initial_sequencer_values(p_inst); 
    transport_gpio_init(p_inst); 
    init_hardware_timers(p_inst, p_cfg);
    init_gate_gpios(p_inst);

 
}

/* Since configuration starts on caller's thread, configure fields that require
 * immediate and/or inconsequential configuration and defer rest to be handled
 * by our own thread later. */
static void config_instance_immediate(
        struct Sequencer_Instance     * p_inst,
        struct Sequencer_Instance_Cfg * p_cfg)
{
    config_instance_queues(p_inst, p_cfg);
}

static void init_instance_lists(struct Sequencer_Instance * p_inst)
{
    for(enum Sequencer_Evt_Sig sig = k_Seq_Evt_Sig_Beg;
                         sig < k_Seq_Evt_Sig_End;
                         sig++){
        sys_slist_init(&p_inst->list.listeners[sig]);
    }
}

static void clear_instance(struct Sequencer_Instance * p_inst)
{
    memset(p_inst, 0, sizeof(*p_inst));
}

static void init_instance(struct Sequencer_Instance * p_inst)
{
    clear_instance(p_inst);
    init_instance_lists(p_inst);
    #if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
    p_inst->err = k_Seq_Err_Id_None;
    #endif
}

/* ************
 * Module Utils
 * ************/

static void init_module_lists(void)
{
    sys_slist_init(&md.list.instances);
}

static void clear_module(void)
{
    memset(&md, 0, sizeof(md));
}

static void init_module(void)
{
    if(md.initialized) return;
    clear_module();
    init_module_lists();
    md.initialized = true;
}

/* **************
 * Error Checking
 * **************/

#if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
static void set_error(
        struct Sequencer_Instance * p_inst,
        enum Sequencer_Err_Id       err,
        bool                  override)
{
    if(p_inst){
        if((override                          )
        || (p_inst->err == k_Seq_Err_Id_None )){
            p_inst->err = err;
        }
    }
}

static bool errored(
        struct Seq_Instance * p_inst,
        enum Seq_Err_Id       err )
{
    set_error(p_inst, err, NO_OVERRIDE);
    return(err != k_Seq_Err_Id_None);
}
#endif /* CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING */

/* ************
 * Broadcasting
 * ************/

static void broadcast(
        struct Sequencer_Evt      * p_evt,
        struct Sequencer_Listener * p_lsnr)
{
    /* call the listener, passing the event */
    if(p_lsnr->cb) p_lsnr->cb(p_evt);
}

static void broadcast_event_to_listeners(
        struct Sequencer_Instance * p_inst,
        struct Sequencer_Evt      * p_evt)
{
    enum Sequencer_Evt_Sig sig = p_evt->sig;
    struct Sequencer_Listener * p_lsnr = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&p_inst->list.listeners[sig], p_lsnr, node.listener){
        broadcast(p_evt, p_lsnr);
    }
}

/* There's only 1 listener for instance initialization, and it is provided in
 * the cfg struct. */
static void broadcast_instance_initialized(
        struct Sequencer_Instance * p_inst,
        Sequencer_Listener_Cb       cb)
{
    struct Sequencer_Evt evt = {
            .sig = k_Seq_Evt_Sig_Instance_Initialized,
            .data.initd.p_inst = p_inst
    };

    struct Sequencer_Listener lsnr = {
        .p_inst = p_inst,
        .cb = cb
    };

    broadcast(&evt, &lsnr);
}

#if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
static void broadcast_interface_deinitialized(
        struct Seq_Instance * p_inst,
        Sequencer_Listener_Cb       cb)
{
    struct Sequencer_Evt evt = {
            .sig = k_Seq_Instance_Deinitialized,
            .data.inst_deinit.p_inst = p_inst
    };

    struct Sequencer_Listener lsnr = {
        .p_inst = p_inst,
        .cb = cb
    };

    broadcast(&evt, &lsnr);
}
#endif



/* **************
 * Listener Utils
 * **************/

static void add_listener_for_signal_to_listener_list(
    struct Sequencer_Listener_Cfg * p_cfg)
{
    /* Get pointer to interface to add listener to. */
    struct Sequencer_Instance * p_inst = p_cfg->p_inst;

    /* Get pointer to configured listener. */
    struct Sequencer_Listener * p_lsnr = p_cfg->p_lsnr;

    /* Get signal to listen for. */
    enum Sequencer_Evt_Sig sig = p_cfg->sig;

    /* Add listener to instance's specified signal. */
    sys_slist_t * p_list = &p_inst->list.listeners[sig];
    sys_snode_t * p_node = &p_lsnr->node.listener;
    sys_slist_append(p_list, p_node);
}

#if CONFIG_FKMG_SEQ_ALLOW_LISTENER_REMOVAL
static bool find_list_containing_listener_and_remove_listener(
    struct Sequencer_Instance * p_inst,
	struct Sequencer_Listener * p_lsnr)
{
    for(enum Sequencer_Evt_Sig sig = k_Seq_Evt_Sig_Beg;
                         sig < k_Seq_Evt_Sig_End;
                         sig++){
        bool found_and_removed = sys_slist_find_and_remove(
                &p_inst->list.listeners[sig], &p_lsnr->node);
        if(found_and_removed) return(true);
    }
    return( false );
}
#endif



/* **************
 * Event Queueing
 * **************/

static void q_sm_event(
        struct Sequencer_Instance * p_inst, struct Sequencer_SM_Evt * p_evt)
{
    bool queued = k_msgq_put(p_inst->msgq.p_sm_evts, p_evt, K_NO_WAIT) == 0;

    if(!queued) assert(false);
}

static void q_init_instance_event(struct Sequencer_Instance_Cfg * p_cfg)
{
    struct Sequencer_SM_Evt evt = {
            .sig = k_Seq_SM_Evt_Sig_Init_Instance,
            .data.init_inst.cfg = *p_cfg
    };
    struct Sequencer_Instance * p_inst = p_cfg->p_inst;
    q_sm_event(p_inst, &evt);
}


static void q_timer_elapsed (struct Sequencer_Instance * p_inst, enum Sequencer_Id id)
{
    struct Sequencer_SM_Evt evt = {
        .sig = k_Seq_SM_Evt_Timer_Elapsed,
        .data.stepped.id = id,
        .data.stepped.edge = p_inst->seq.edge[id]
    };
    q_sm_event(p_inst, &evt);

}


static void timer_x_callback (const struct device *timer_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data) {

    struct Sequencer_Instance *p_inst = (struct Sequencer_Instance *)user_data; 

    q_timer_elapsed(p_inst, k_Seq_Id_1); 

}

static void timer_y_callback (const struct device *timer_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data) {
    
    struct Sequencer_Instance *p_inst = (struct Sequencer_Instance *)user_data; 

    q_timer_elapsed(p_inst, k_Seq_Id_2); 

}


/* **********
 * Listener Callbacks
 * **********/


/* **********
 * SM Functions
 * **********/


static void post_updated_pot_value(struct Sequencer_Instance * p_inst, enum Pot_Id id, uint16_t val) 
{
    if (id < 16) {
        p_inst->seq.voltage[id] = val; 
    } else if (id < 32) {
        p_inst->seq.time[ id - 16 ] = val; 
    } else {
        switch(id) {
            default: break; 
            case 32:
            case 33:
                p_inst->seq.param[ id - 32 ] = val; 
                break; 
            case 34: 
                p_inst->seq.global = val; 
                break; 
        }
    }
}


/*
 * Gets called state machine after SM receives pot change event and posts new value to our instance. Main function is to decide whether or not
 * we need to make any immediate changes to our voltage or timer values. If a step is currently active and outputting to the dac as the pot is changed,
 * we'll want to reflect that in real time. As all voltages and time values are grouped together within the sequencer instance struct, 
 * we need to identify which pot belongs to which sequencer, then whether that pot is a time value or voltage value. 
*/

static void check_current_step(struct Sequencer_Instance * p_inst, enum Pot_Id id) 
{
    uint8_t ch;
    uint8_t stp; 

    switch (id) {
        case 32:
        case 33:
            /* these two pots deal with global time scale for each sequencer channel, 
            so if we detect a change here we'll have to update our timers as well */
            return;
            break;
        case 34:
            /* Some parameter, not quite sure what it does yet, 
            might be clock divider. Only time will tell. */
            return; 
            break; 
        default:  // default handles all time and voltage pots
            if (id > 0 && id <= 31) {

                /* if the pot Id coming in is greater than the max steps of sequencer 1, we know pot is in sequencer 2 
                   note the channel offset is to account for the fact that the dac api channels begin at 1, while our arrays begin at 0.  
                */
                ch = (id > p_inst->seq.maxStep[0]) ? 2 : 1; 
                stp = id % p_inst->seq.maxStep[ ch == 1 ? 0 : 1]; 

                if (stp != p_inst->seq.step[ch-1]) {
                    return; 
                }

                /* If the changed pot corresponds to an active step we need to send a message to change that value */

                if (id == (stp + 16)) {
                    /*
                        Deal with a timer change.
                        If new timer value is less than what's left on the next step, we force sequencer to the next step.
                        If it's greater than what's left we subtract the elapsed time so far from the new time and set timer. 
                        If we're within a couple of ms, it's probably not worth changing? 
                    */
                } else {
                    dac_write_new_value(ch, p_inst->seq.voltage[stp]);
                }

            } else {

                return; 
            }
            break;

    }
  

}


static void set_voltage_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    uint8_t channel = id + 1;

    dac_write_new_value(channel, p_inst->seq.voltage[p_inst->seq.step[id]]); 

}

static void set_gate_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, bool status) 
{
    if (id < 0 || id >= NUMBER_OF_GATES) return;

    int result = gpio_pin_set_dt(&gates[id], status);

    if (result < 0) printk("Gate %d Error: %d\n", id, result);
}

/*
 * TODO: does Sequencer thread keep only to sending messages in this case, or do we handle actual MIDI calcs here?
 * Depends on whether or not uart module is uart in general or specifically a midi module? The latter might make more sense for
 * my general purposes. In that case, only message that something happened goes to module. (any sync issues to be aware of?)
*/
static void set_midi_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, uint8_t step, uint8_t offset) 
{

    uint8_t midi_ch_base = p_inst->seq.edge[id] ? 0x80 : 0x90; 

    struct Sequencer_Evt_MIDI_Write_Ready evt_cfg = {
        .id = id,
        .midi_status = midi_ch_base | id,
        .ctrl_byte = 0x75,
        .seq = (bool)id,
        .step = step,
        .offset = p_inst->seq.offset
    };

    struct Sequencer_Evt evt = {
        .sig = k_UART_Evt_Sig_Write_Ready,
        .data.midi_write = evt_cfg
    };

    broadcast_event_to_listeners(p_inst, &evt);
}

/*
 * Tell UI module (here named led_driver) only which LED we need to hit high. The module itself will
 * keep track of everything else that needs to stay dimmed and high from other calls (e.g. the other sequencer). 
*/
// static void set_ui_on_step (struct Sequencer_Instance * p_inst, 
//                             enum Sequencer_Id id,
//                             uint16_t step, 
//                             uint16_t offset, 
//                             uint16_t val) 
// {
//     struct Sequencer_Evt_LED_Write_Ready evt_cfg = {
//         .id = id,
//         .step = step,
//         .offset = offset,
//         .val = val
//     }; 

//     struct Sequencer_Evt evt = {
//         .sig = k_Seq_Evt_Sig_LED_Write_Ready,
//         .data.led_write = evt_cfg
//     };

//     broadcast_event_to_listeners(p_inst, &evt);
// }


static void advance_sequencer_step (struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{

    if (p_inst->seq.maxStep[id] == 0) return;

    uint8_t next_step = (p_inst->seq.step[id] + 1) % p_inst->seq.maxStep[id];

    p_inst->seq.step[id] = next_step; 

}

bool play_pause_1_led_track = false;
bool play_pause_1_transport_track = false;
static uint64_t play_pause_1_last_time = 0;

bool play_pause_2_led_track = false;
bool play_pause_2_transport_track = false;
static uint64_t play_pause_2_last_time = 0;

bool reset_1_led_track = false;
bool reset_1_transport_track = false;
static uint64_t reset_1_last_time = 0;

bool reset_2_led_track = false;
bool reset_2_transport_track = false;
static uint64_t reset_2_last_time = 0;


static void on_transportA(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance *p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_1_cb);

    static uint64_t last_time = 0;
    uint64_t current_time = k_uptime_get();
    if (current_time - last_time < DEBOUNCE_TIME_MS) return;
    last_time = current_time;

    bool pin_state = gpio_pin_get(playPauseA.port, playPauseA.pin);

    if (pin_state) {
        play_pause_1_led_track = !play_pause_1_led_track;
        gpio_pin_set(play_led_high.port, play_led_high.pin, play_pause_1_led_track);

        if (p_inst->seq.running[0]) {
            counter_stop(p_inst->timer.t[0]);
        } else {
            counter_start(p_inst->timer.t[0]);
        }
        p_inst->seq.running[0] = !p_inst->seq.running[0];
    }
}



static void on_transportB(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance *p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_2_cb);

    static uint64_t last_time = 0;
    uint64_t current_time = k_uptime_get();
    if (current_time - last_time < DEBOUNCE_TIME_MS) return;
    last_time = current_time;

    bool pin_state = gpio_pin_get(playPauseB.port, playPauseB.pin);

    if (pin_state) {
        play_pause_2_led_track = !play_pause_2_led_track;
        gpio_pin_set(play_led_low.port, play_led_low.pin, play_pause_2_led_track);

        if (p_inst->seq.running[1]) {
            counter_stop(p_inst->timer.t[1]);
        } else {
            counter_start(p_inst->timer.t[1]);
        }
        p_inst->seq.running[1] = !p_inst->seq.running[1];
    }
}

static void on_resetA(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance * p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_1_rst_cb);

    /* Debounce */
    uint64_t current_time = k_uptime_get();
    if (current_time - reset_1_last_time < 50) return;
    reset_1_last_time = current_time;

    bool pin_state = gpio_pin_get(resetA.port, resetA.pin);

    if (pin_state) {
        /* Reset sequencer step to 0 */
        p_inst->seq.step[p_inst->id] = 0;
    } 

    gpio_pin_set(reset_led_high.port, reset_led_high.pin, pin_state);

}

static void on_resetB(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance * p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_2_rst_cb);

    /* Debounce */
    uint64_t current_time = k_uptime_get();
    if (current_time - reset_2_last_time < 50) return;
    reset_2_last_time = current_time;

    bool pin_state = gpio_pin_get(resetB.port, resetB.pin);

    if (pin_state) {
        /* Reset sequencer step to 0 */
        p_inst->seq.step[p_inst->id] = 0;
    } 

    gpio_pin_set(reset_led_low.port, reset_led_low.pin, pin_state);
}

/* **********
 * FSM States
 * **********/

/* The states are flat:
 *
 *  State init:
 *
 *  |entry          |  |run                |  |exit           |
 *  |---------------|  |-------------------|  |---------------|
 *  |not implemented|  |init instance:     |  |not implemented|
 *                     |* finish cfg.      |
 *                     |* ->run            
 *                     |all other signals  |
 *                     |not allowed; assert|
 *
 *  State run:
 *
 *  |entry             |  |run             |  |exit           |
 *  |------------------|  |----------------|  |---------------|
 *  |* start conversion|  |convert:        |  |not implemented|
 *  |  timer           |  |* timer elapsed |
 *                        |* pot changed   |
 *                        |* btn changed   |
 *                        |deinit:         |
 *                        |* ->deinit      |
 *                        |all others:     |
 *                        |* assert        |
 *
 *  State deinit:
 *
 *  |entry             |  |run                 |  |exit           |
 *  |------------------|  |--------------------|  |---------------|
 *  |* start conversion|  |* finish dcfg.      |  |not implemented|
 *  |  timer           |  |* exit state machine|
 *                        |all others:         |
 *                        |* assert            |
 *
 *  Legend:
 *  "entry state": 1st state of state machine
 *  "xxx:"       : signal to handle
 *  "*"          : action to take on signal
 *  "-> xxx"     : state to change to
 *  "^ xxx"      : pass signal to parent, with expected parent action on signal.
 */


/* Forward declaration of state table */
static const struct smf_state states[];

enum state{
    init,   // Init instance - should only occur once, after thread start
    run,    // Run - handles all events while running (e.g. conversion, etc.)
    deinit, // Deinit instance - should only occur once, after deinit event
            // (if implemented)
};

/* NOTE: in all the state functions the param o is a pointer to the state
 * machine context, which is &p_inst->sm. */

/* Init state responsibility is to complete initialization of the instance, let
 * an instance config listener know we've completed initialization (i.e. thread
 * is up and running), and then transition to next state. Init state occurs
 * immediately after thread start and is expected to only occur once. */

static void state_init_run(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Sequencer_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct Sequencer_SM_Evt * p_evt = &p_inst->sm_evt;

    /* Expecting only an "init instance" event. Anything else is an error. */
    assert(p_evt->sig == k_Seq_SM_Evt_Sig_Init_Instance);

    /* We init'd required params on the caller's thread (i.e.
     * Sequencer_Init_Instance()), now finish the job. Since this is an
     * Init_Instance event the data contains the intance cfg. */
    struct Sequencer_SM_Evt_Sig_Init_Instance * p_ii = &p_evt->data.init_inst;

    config_instance_deferred(p_inst, &p_ii->cfg);
    broadcast_instance_initialized(p_inst, p_ii->cfg.cb);  
    smf_set_state(SMF_CTX(p_sm), &states[run]);
}

/* Run state responsibility is to be the root state to handle everything else
 * other than instance initialization. */

static void state_run_entry(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Sequencer_Instance * p_inst = sm_ctx_to_instance(p_sm);
}

static void state_run_run(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Sequencer_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct Sequencer_SM_Evt * p_evt = &p_inst->sm_evt;

    switch(p_evt->sig){
        default: break;
        case k_Seq_SM_Evt_Sig_Init_Instance:
            /* Should never occur. */
            assert(false);
            break;
        case k_Seq_SM_Evt_Timer_Elapsed:

            struct Sequencer_SM_Evt_Sig_Timer_Elapsed * p_stepped = &p_evt->data.stepped; 
            uint8_t ledval;
            uint16_t new_tix; 
            enum Sequencer_Id id = p_stepped->id;
            switch(p_stepped->edge) {

                case FALLING_EDGE:

                    update_leds(id, p_inst->seq.step[id], p_inst->seq.offset, EIGHTH_BRIGHTNESS);
                    set_voltage_on_step(p_inst, id);
                    set_gate_on_step(p_inst, id, true);
                    
                    p_inst->seq.edge[id] = RISING_EDGE;
                    
                    new_tix = calculate_gate_timer_delay(p_inst, id, RISING_EDGE);
                    
                    if (new_tix < 1) new_tix = 1;
                    
                    reset_timer(p_inst, id, new_tix);
                break;

                case RISING_EDGE:

                    advance_sequencer_step(p_inst, id);
                    set_gate_on_step(p_inst, id, false);
                    
                    p_inst->seq.edge[id] = FALLING_EDGE;
                    
                    new_tix = calculate_gate_timer_delay(p_inst, id, FALLING_EDGE);
                    
                    if (new_tix < 1) new_tix = 1;
                    
                    reset_timer(p_inst, id, new_tix);
                break; 
            }

                        // if(p_stepped->edge == FALLING_EDGE) {
                        //     update_leds(id, p_inst->seq.step[id], p_inst->seq.offset, EIGHTH_BRIGHTNESS);
                        //     set_voltage_on_step(p_inst, id);
                        //     set_gate_on_step(p_inst, id, true);
                        //     p_inst->seq.edge[id] = RISING_EDGE;
                        //     new_tix = calculate_gate_timer_delay(p_inst, id, RISING_EDGE);
                        //     if (new_tix < 1) new_tix = 1;
                        //     reset_timer(p_inst, id, new_tix);
                        // } else {
                        //     advance_sequencer_step(p_inst, id);
                        //     set_gate_on_step(p_inst, id, false);
                        //     p_inst->seq.edge[id] = FALLING_EDGE;
                        //     new_tix = calculate_gate_timer_delay(p_inst, id, FALLING_EDGE);
                        //     if (new_tix < 1) new_tix = 1;
                        //     reset_timer(p_inst, id, new_tix);
                        // }

            break;

        case k_Seq_SM_Evt_Sig_Pot_Value_Changed:
            struct Sequencer_SM_Evt_Sig_Pot_Value_Changed * p_pot_changed = &p_evt->data.pot_changed; 
            
            // post_updated_pot_value(p_inst, p_pot_changed->pot_id, p_pot_changed->pot_id);
            // check_current_step(p_inst, p_pot_changed->pot_id);

            break;
        case k_Seq_SM_Evt_Sig_Button_Pressed:
            struct Sequencer_SM_Evt_Sig_Button_Pressed * p_button = &p_evt->data.btn_pressed; 
            #if 0 /* Pseudo code: */
            Buttons in normal state tell us which steps are active. In this case, just update sequencer instance information with which steps are active. 
            Later we will deal with the mode button which, when held, will put us in another state to edit the length of our sequences. 
            #endif
            break; 
        #if CONFIG_FKMG_SEQ_SHUTDOWN_ENABLED
        case k_Seq_Evt_Sig_Instance_Deinitialized:
            assert(false);
            break;
        #endif
    }
}

/* Deinit state responsibility is to clean up before exiting thread. */
#if CONFIG_FKMG_SEQ_SHUTDOWN_ENABLED
static void state_deinit_run(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Sequencer_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct Sequencer_SM_Evt * p_evt = &p_inst->sm_evt;

    /* TODO */
}
#endif

static const struct smf_state states[] = {
    /*                                      entry               run  exit */
    [  init] = SMF_CREATE_STATE(           NULL,   state_init_run, NULL),
    [   run] = SMF_CREATE_STATE(state_run_entry,    state_run_run, NULL),
    #if CONFIG_FKMG_SEQ_SHUTDOWN_ENABLED
    [deinit] = SMF_CREATE_STATE(           NULL, state_deinit_run, NULL),
    #endif
};

/* ******
 * Thread
 * ******/

#if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
/* Since there is only the "join" facility to know when a thread is shut down,
 * and that isn't appropriate to use since it will put the calling thread to
 * sleep until the other thread is shut down, we set up a delayable system work
 * queue event to check that the thread is shut down and then call any callback
 * that is waiting to be notified. */
void on_thread_shutdown(struct k_work *item)
{
    struct Sequencer_Instance * p_inst =
            CONTAINER_OF(item, struct Sequencer_Instance, work);

    char * thread_state_str = "dead";
    k_thread_state_str(&p_inst->thread, thread_state_str, sizeof(thread_state_str));
    bool shut_down = strcmp( thread_state_str, "dead" ) == 0;

    if(!shut_down) k_work_reschedule(&p_inst->work, K_MSEC(1));
    else broadcast_instance_deinitialized(p_inst);
}
#endif

static void thread(void * p_1, /* struct Sequencer_Instance* */
        void * p_2_unused, void * p_3_unused)
{
    struct Sequencer_Instance * p_inst = p_1;
    /* NOTE: smf_set_initial() executes the entry state. */
    struct smf_ctx * p_sm = &p_inst->sm;
    smf_set_initial(SMF_CTX(p_sm), &states[init]);

    /* Get the state machine event queue and point to where to put the dequeued
     * event. */
    struct k_msgq * p_msgq = p_inst->msgq.p_sm_evts;
    struct Sequencer_SM_Evt * p_evt = &p_inst->sm_evt;

    bool run = true;

    while(run){
        /* Wait on state machine event. Cache it then run state machine. */
        k_msgq_get(p_msgq, p_evt, K_FOREVER);
        run = smf_run_state(SMF_CTX(p_sm)) == 0;
    }

    #if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
    /* We're shutting down. Schedule a work queue event to check that the
     * thread exited and call back anything. */
    if(should_callback_on_exit(p_inst)){
        k_work_init_delayable( &p_inst->work, on_thread_shutdown);
        k_work_schedule(&p_inst->work, K_MSEC(1));
    }
    #endif
}

static void start_thread(
        struct Sequencer_Instance     * p_inst,
        struct Sequencer_Instance_Cfg * p_cfg)
{
    struct k_thread  * p_thread = p_cfg->task.sm.p_thread;
    k_thread_stack_t * p_stack  = p_cfg->task.sm.p_stack;
    size_t stack_sz_bytes = p_cfg->task.sm.stack_sz;
    int prio = p_cfg->task.sm.prio;

    /* Start the state machine thread. */
    p_inst->task.sm.p_thread = k_thread_create(
        p_thread,       // ptr to uninitialized struct k_thread
        p_stack,        // ptr to the stack space
        stack_sz_bytes, // stack size in bytes
        thread,         // thread entry function
        p_inst,         // 1st entry point parameter
        NULL,           // 2nd entry point parameter
        NULL,           // 3rd entry point parameter
        prio,           // thread priority 
        0,              // thread options
        K_NO_WAIT);     // scheduling delay
    assert(p_inst->task.sm.p_thread == p_thread);
}

/* *****************************************************************************
 * Public
 */




void save_channel_state(uint16_t state) {
    struct nvs_fs fs;
    int ret;

    // /* Initialize NVS */
    // ret = nvs_init(&fs, DT_LABEL(DT_ALIAS(nvs_flash)));
    // if (ret) {
    //     LOG_ERR("Error: NVS Init");
    // }

    // /* Write state to NVS */
    // ret = nvs_write(&fs, CHANNEL_STATE_ID, &state, sizeof(state));
    // if (ret < 0) {
    //     LOG_ERR("Error: NVS Write");
    // }
}

uint16_t load_channel_state(void) {
    struct nvs_fs fs;
    uint16_t state = 0;
    int ret;

    // /* Initialize NVS */
    // ret = nvs_init(&fs, DT_LABEL(DT_ALIAS(nvs_flash)));
    // if (ret) {
    //     LOG_ERR("Error: NVS Init");
    // }

    // /* Read state from NVS */
    // ret = nvs_read(&fs, CHANNEL_STATE_ID, &state, sizeof(state));
    // if (ret < 0) {
    //     LOG_ERR("Error: NVS Read");
    // }

    return state;
}


void Sequencer_Init_Instance(struct Sequencer_Instance_Cfg * p_cfg)
{
    /* Get pointer to instance to configure. */
    struct Sequencer_Instance * p_inst = p_cfg->p_inst;

    #if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
    /* Check instance configuration for errors. */
    if(errored(p_inst, check_instance_cfg_param_for_errors(p_cfg))){
        assert(false);
        return;
    }
    #endif

    init_module();
    init_instance(p_inst);
    config_instance_immediate(p_inst, p_cfg);
    add_instance_to_instances(p_inst);
    start_thread(p_inst, p_cfg);
    q_init_instance_event(p_cfg);
}

#if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
void Sequencer_Deinit_Instance(struct Sequencer_Instance_Dcfg * p_dcfg)
{
    #error "Not implemented yet!"
}
#endif

void Sequencer_Add_Listener(struct Sequencer_Listener_Cfg * p_cfg)
{
    #if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
    /* Get pointer to instance. */
    struct Sequencer_Instance * p_inst = p_cfg->p_inst;

    /* Check listener instance configuration for errors. */
    if(errored(p_inst, check_listener_cfg_param_for_errors(p_cfg))){
        assert(false);
        return;
    }
    #endif

    struct Sequencer_Listener * p_lsnr = p_cfg->p_lsnr;
    init_listener(p_lsnr);
    config_listener(p_lsnr, p_cfg);
    add_listener_for_signal_to_listener_list(p_cfg);
}

#if CONFIG_FKMG_SEQ_ALLOW_LISTENER_REMOVAL
void Sequencer_Remove_Listener(struct Sequencer_Listener * p_lsnr)
{ 
    #error "Not implemented yet!"
}
#endif

#if CONFIG_FKMG_SEQ_NO_OPTIMIZATIONS
#pragma GCC pop_options
#endif
