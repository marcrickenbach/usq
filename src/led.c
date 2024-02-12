/* *****************************************************************************
 * @brief LED Driver implementation.
 */

/* *****************************************************************************
 * TODO
 */

/* *****************************************************************************
 * Includes
 */

#include "led.h"

#include <zephyr/smf.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/dma.h>

#include <assert.h>

#include <stdlib.h>

#include "led/private/sm_evt.h"
#include "led/private/module_data.h"

/* *****************************************************************************
 * Constants, Defines, and Macros
 */

#define SUCCESS (0)
#define FAIL    (-1)

#define OVERRIDE    true
#define NO_OVERRIDE false

#define SPI2_NODE       DT_NODELABEL(spi2)
#define PWM_LED_NODE    DT_NODELABEL(led_pwm)
#define PWM_CTLR DT_PWMS_CTLR(PWM_LED_NODE)
#define PWM_CHAN DT_PWMS_CHANNEL(PWM_LED_NODE)
#define PWM_FLAGS DT_PWMS_FLAGS(PWM_LED_NODE)

#define GPIO_PINS           DT_PATH(zephyr_user)

#define LED_BLANK_TIMER     DT_INST(2, st_stm32_counter)

/* LED Brightness */
#define FULL_BRIGHTNESS     0xFFF
#define HALF_BRIGHTNESS     0x7FF
#define QUARTER_BRIGHTNESS  0x3FF
#define EIGHTH_BRIGHTNESS   0x1FF
#define DIM_ARMED_STEP      0x4B

#define NO_OFFSET           16

static bool led_animated_complete = false;

/* *****************************************************************************
 * Debugging
 */

#if CONFIG_FKMG_LED_DRIVER_NO_OPTIMIZATIONS
#pragma GCC push_options
#pragma GCC optimize ("Og")
#warning "led.c unoptimized!"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LED_Driver);

/* *****************************************************************************
 * Structs
 */

static struct LED_Driver_module_data led_driver_md = {0};
/* Convenience accessor to keep name short: md - module data. */
#define md led_driver_md


static const struct device * spi_dev = DEVICE_DT_GET(SPI2_NODE);
static const struct device * led_pwm_dev = DEVICE_DT_GET(PWM_LED_NODE);

static const struct gpio_dt_spec led_le = GPIO_DT_SPEC_GET(GPIO_PINS, led_le_gpios); 

static const struct gpio_dt_spec mode_led      = GPIO_DT_SPEC_GET(GPIO_PINS, led_21_gpios);

static const struct device *led_driver_blank_timer = DEVICE_DT_GET(LED_BLANK_TIMER);
static const struct gpio_dt_spec led_blank = GPIO_DT_SPEC_GET(GPIO_PINS, led_blank_gpios);

const static int led_to_out_map[16] = {15, 14, 13, 12, 11, 10, 9, 8, 0, 1, 2, 3, 4, 5, 6, 7};

/* Buffer to hold LED data during our run. Two public APIs, update_leds and arm_leds (maybe more in future) will read and write to this variable.
 * This might be an issue down the line, keep an eye on it. Don't necessarily want to be reading and writing to this buffer from multiple threads even if we pass through this one.
*/
static uint16_t led_buffer[16] = {0};
static bool led_armed[16] = {0};

/* Final LED Data to output */
static uint8_t led_data_output[24] = {0};

struct spi_config spi_cfg = {
    .frequency = 2500000,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
    .slave = 0,
    .cs = NULL,
};

struct spi_buf tx_buf = {
    .buf = led_data_output,
    .len = sizeof(led_data_output)
};

struct spi_buf_set tx = {
    .buffers = &tx_buf,
    .count = 1
};



/* *****************************************************************************
 * Private
 */

/* *****
 * Utils
 * *****/

static bool instance_contains_state_machine_ctx(
        struct LED_Driver_Instance * p_inst,
        struct smf_ctx      * p_sm_ctx)
{
    return(&p_inst->sm == p_sm_ctx);
}

/* Find the instance that contains the state machine context. */
static struct LED_Driver_Instance * sm_ctx_to_instance(struct smf_ctx * p_sm_ctx)
{
    /* Iterate thru the instances. */
    struct LED_Driver_Instance * p_inst = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&md.list.instances, p_inst, node.instance){
        if(instance_contains_state_machine_ctx(p_inst, p_sm_ctx)) return p_inst;
    }
    return(NULL);
}



/* **************
 * Listener Utils
 * **************/

static void clear_listener(struct LED_Driver_Listener * p_lsnr)
{
    memset(p_lsnr, 0, sizeof(*p_lsnr));
}

static void config_listener(
        struct LED_Driver_Listener     * p_lsnr,
        struct LED_Driver_Listener_Cfg * p_cfg)
{
    /* Set listner's instance it is listening to. */
    p_lsnr->p_inst = p_cfg->p_inst;

    /* Set listner's callback. */ 
    p_lsnr->cb = p_cfg->cb;
}

static void init_listener(struct LED_Driver_Listener * p_lsnr)
{
    clear_listener(p_lsnr);
}

/* **************
 * Instance Utils
 * **************/


static void add_instance_to_instances(
        struct LED_Driver_Instance  * p_inst)
{
    sys_slist_append(&md.list.instances, &p_inst->node.instance);
}

static void config_instance_queues(
        struct LED_Driver_Instance     * p_inst,
        struct LED_Driver_Instance_Cfg * p_cfg)
{
    p_inst->msgq.p_sm_evts = p_cfg->msgq.p_sm_evts;
}


static void config_instance_deferred(
        struct LED_Driver_Instance     * p_inst,
        struct LED_Driver_Instance_Cfg * p_cfg)
{

}

/* Since configuration starts on caller's thread, configure fields that require
 * immediate and/or inconsequential configuration and defer rest to be handled
 * by our own thread later. */
static void config_instance_immediate(
        struct LED_Driver_Instance     * p_inst,
        struct LED_Driver_Instance_Cfg * p_cfg)
{
    config_instance_queues(p_inst, p_cfg);
}

static void init_instance_lists(struct LED_Driver_Instance * p_inst)
{
    for(enum LED_Driver_Evt_Sig sig = k_LED_Driver_Evt_Sig_Beg;
                         sig < k_LED_Driver_Evt_Sig_End;
                         sig++){
        sys_slist_init(&p_inst->list.listeners[sig]);
    }
}



static void clear_instance(struct LED_Driver_Instance * p_inst)
{
    memset(p_inst, 0, sizeof(*p_inst));
}

static void init_instance(struct LED_Driver_Instance * p_inst)
{
    clear_instance(p_inst);
    init_instance_lists(p_inst);
}

/****************************
 * LED DRIVER FUNCTIONS
 ****************************/


static void led_blank_pulse(bool status) 
{
    int ret = gpio_pin_set(led_blank.port, led_blank.pin, status);
    ret = gpio_pin_set(led_blank.port, led_blank.pin, !status);
    if (ret < 0) {
        LOG_ERR("Could not set LED latch enable pin");
        return;
    }
}

static void led_blank_timer_callback (const struct device *timer_dev,
                              uint8_t chan_id, uint32_t ticks,
                              void *user_data)
{

    led_blank_pulse(true);

    #define DESIRED_TIME_IN_MS 4
    //1MHz is counter frequency, we're counting to 4000 ticks, so that we have time to fire the interrupt and hit around 4095
    
    uint32_t tix = (DESIRED_TIME_IN_MS * 1000000) / 1000;

    struct counter_alarm_cfg blank_cfg = {
        .flags = 0,
        .ticks = tix,
        .callback = led_blank_timer_callback,
        .user_data = user_data
    }; 

    
    if (counter_set_channel_alarm(led_driver_blank_timer, 0, &blank_cfg) != 0) {
        LOG_ERR("Error: Failed to set LED Driver Blank Timer"); 
        return;
    }

    return; 
}

static void init_led_blank_timer(struct LED_Driver_Instance * p_inst)
{
    if (!device_is_ready(led_driver_blank_timer)){
        LOG_ERR("LED Blank Timer Config: FAILED.\n");
        return;
    }

    #define DESIRED_TIME_IN_MS 4

    uint32_t ticks = (DESIRED_TIME_IN_MS * 1000000) / 1000;

    struct counter_alarm_cfg blank_cfg = {
        .flags = 0,
        .ticks = ticks,
        .callback = led_blank_timer_callback,
        .user_data = p_inst
    };

    
    if (counter_set_channel_alarm(led_driver_blank_timer, 0, &blank_cfg) != 0) {
        LOG_ERR("Error: Failed to set LED Driver Blank Timer"); 
    }

    return; 

}


static void led_latch_enable_pulse(bool status) 
{
    int ret = gpio_pin_set(led_le.port, led_le.pin, status);
    ret = gpio_pin_set(led_le.port, led_le.pin, !status);

    if (ret < 0) {
        LOG_ERR("Could not set LED latch enable pin");
        return;
    }
}


void dma_spi_tx_callback(const struct device *dma_dev, void *user_data, uint32_t channel, int status) {
    
    led_latch_enable_pulse(true);

}

static void pwm_signal_init(struct LED_Driver_Instance * p_inst) {
    
    // 1Mhz, 50% duty cycle
    int ret = pwm_set(led_pwm_dev, 1, 1000, 500, 0);
    assert(ret == 0);

}

static void led_blank_init(struct LED_Driver_Instance * p_inst) 
{
    int ret = gpio_pin_configure_dt(&led_blank, GPIO_OUTPUT_ACTIVE);
    
    if (ret < 0) {
        LOG_ERR("Could not configure LED latch enable pin, %d", ret);
        return;
    }
}


static void led_latch_enable_init(struct LED_Driver_Instance * p_inst) 
{
    int ret = gpio_pin_configure_dt(&led_le, GPIO_OUTPUT_INACTIVE);

    if (ret < 0) {
        LOG_ERR("Could not configure LED latch enable pin, %d", ret);
        return;
    }

}


static void led_driver_init(struct LED_Driver_Instance * p_inst) 
{

    if (!device_is_ready(mode_led.port)) {
        LOG_ERR("Could not get mode led");
        return;
    }

    int ret = gpio_pin_configure_dt(&mode_led, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }

    uint8_t dummy_data[25] = {0};

    struct spi_buf dummy_buf = {
        .buf = dummy_data,
        .len = sizeof(dummy_data)
    };

    struct spi_buf_set dummy_spi_tx = {
        .buffers = &dummy_buf,
        .count = 1
    };

    struct spi_buf_set dummy_spi_rx = {
        .buffers = &dummy_buf,
        .count = 1
    };

    ret = device_is_ready(spi_dev); 
    if (ret == false) {
        LOG_ERR("Could not get SPI device");
        return;
    }

    if (led_pwm_dev == NULL || !device_is_ready(led_pwm_dev)) {
        LOG_ERR("Could not get PWM device");
        return;
    }

    // Initialize LED Driver Latch Enable Pin, Blank Pin and Grayscale Clock (PWM)
    led_blank_init(p_inst);
    led_latch_enable_init(p_inst);
    pwm_signal_init(p_inst);
    init_led_blank_timer(p_inst);

    ret = counter_start(led_driver_blank_timer);
    if(ret != 0) {
        LOG_ERR("Error: Failed to start LED Driver Blank Timer"); 
        return;
    }

    k_msleep(100); 

    ret = spi_write(spi_dev, &spi_cfg, &dummy_spi_tx);
    if (ret != 0) {
        LOG_ERR("Could not write to SPI device");
        return;
    }

    led_latch_enable_pulse(true);
}


#define FULL_BRIGHTNESS 0xFFF // 12-bit full brightness
#define STEPS 1200 // Number of steps for brightening and dimming
#define BLINK_DELAY_MS 25 // Delay in milliseconds for blinks


void clear_leds(); 
void set_led_full_brightness(int led_number); 

// Helper function to pack the led_buffer into led_data and write it to the TLC5940
void pack_and_write_led_data() {

    if (led_animated_complete) {
        // transfer the led_armed buffer to the led_buffer
        for (int i = 0; i < 16; i++) {
            if (led_armed[i]) {
                led_buffer[i] = DIM_ARMED_STEP;
            } else {
                led_buffer[i] = 0;
            }
        }
    }

    int j = 0; // Start index for led_data
    for (int m = 0; m < 16; m += 2) {
        led_data_output[j] = led_buffer[m] >> 4;
        led_data_output[j+1] = (led_buffer[m] << 4) | (led_buffer[m + 1] >> 8);
        led_data_output[j+2] = led_buffer[m + 1] & 0xFF;
        j += 3;
    }

    // Write the new led_data to the TLC5940
    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret != 0) {
        LOG_ERR("Could not write to SPI device");
        return;
    }

    // Pulse the latch to update the LED outputs
    led_latch_enable_pulse(true);
}

void led_startup_animation(int duration_ms) 
{

    int brightness_change_per_step = HALF_BRIGHTNESS / STEPS;
    int delay_per_step = duration_ms / (STEPS * 2);


    for (int i = 0; i < 16; i++) {
        // Light up LED
        set_led_full_brightness(i);
        k_msleep(BLINK_DELAY_MS);

        // Blink off
        clear_leds();
        k_msleep(BLINK_DELAY_MS);

    }

    for (int step = 0; step <= STEPS; step++) {
        int current_brightness = brightness_change_per_step * step;

        for (int k = 0; k < 16; k++) {
            led_buffer[k] = current_brightness;
        }
        
        pack_and_write_led_data();
        k_msleep(delay_per_step);
    }

    // Dim the LEDs gradually
    for (int step = STEPS; step >= 0; step--) {
        int current_brightness = brightness_change_per_step * step;

        // Set the brightness for each LED
        for (int k = 0; k < 16; k++) {
            led_buffer[k] = current_brightness;
        }
        
        pack_and_write_led_data();
        k_msleep(delay_per_step);
    }

    led_animated_complete = true;

    // set_initial_armed_leds();
    
    clear_leds();

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


/* ************
 * Broadcasting
 * ************/

static void broadcast(
        struct LED_Driver_Evt      * p_evt,
        struct LED_Driver_Listener * p_lsnr)
{
    /* call the listener, passing the event */
    if(p_lsnr->cb) p_lsnr->cb(p_evt);

}

static void broadcast_event_to_listeners(
        struct LED_Driver_Instance * p_inst,
        struct LED_Driver_Evt      * p_evt)
{

    enum LED_Driver_Evt_Sig sig = p_evt->sig;
    struct LED_Driver_Listener * p_lsnr = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&p_inst->list.listeners[sig], p_lsnr, node.listener){
         broadcast(p_evt, p_lsnr);
    }

}

/* There's only 1 listener for instance initialization, and it is provided in
 * the cfg struct. */
static void broadcast_instance_initialized(
        struct LED_Driver_Instance * p_inst,
        LED_Driver_Listener_Cb       cb)
{
    struct LED_Driver_Evt evt = {
            .sig = k_LED_Driver_Evt_Sig_Instance_Initialized,
            .data.initd.p_inst = p_inst
    };

    struct LED_Driver_Listener lsnr = {
        .p_inst = p_inst,
        .cb = cb
    };

    broadcast(&evt, &lsnr);
}



/* **************
 * Listener Utils
 * **************/

static void add_listener_for_signal_to_listener_list(
    struct LED_Driver_Listener_Cfg * p_cfg)
{
    /* Get pointer to interface to add listener to. */
    struct LED_Driver_Instance * p_inst = p_cfg->p_inst;

    /* Get pointer to configured listener. */
    struct LED_Driver_Listener * p_lsnr = p_cfg->p_lsnr;

    /* Get signal to listen for. */
    enum LED_Driver_Evt_Sig sig = p_cfg->sig;

    /* Add listener to instance's specified signal. */
    sys_slist_t * p_list = &p_inst->list.listeners[sig];
    sys_snode_t * p_node = &p_lsnr->node.listener;
    sys_slist_append(p_list, p_node);
}



/* **************
 * Event Queueing
 * **************/

static void q_sm_event(struct LED_Driver_Instance * p_inst, struct LED_Driver_SM_Evt * p_evt)
{
    bool queued = k_msgq_put(p_inst->msgq.p_sm_evts, p_evt, K_NO_WAIT) == 0;

    if(!queued) assert(false);
}


static void q_init_instance_event(struct LED_Driver_Instance_Cfg * p_cfg)
{
    struct LED_Driver_SM_Evt evt = {
            .sig = k_LED_Driver_SM_Evt_Sig_Init_Instance,
            .data.init_inst.cfg = *p_cfg
    };
    struct LED_Driver_Instance * p_inst = p_cfg->p_inst;

    q_sm_event(p_inst, &evt);
}


static bool armed_leds[16] = {0};

static void buffer_armed_steps_leds(bool channel, uint8_t step, uint8_t offset, enum LED_STATE state)
{
    if (state != LED_STEP) {
        armed_leds[led_to_out_map[step + channel * offset]] = state;
    }

    for (int i = 0; i < 16; i++) {
        if (armed_leds[i]) {
            led_buffer[i] = DIM_ARMED_STEP;
        } else {
            led_buffer[i] = 0;
        }
    }

}

static void map_led_buffer_to_24_bytes(int st_idx, int st_led, int stp_led)
{
    /* Fill the led_data buffer in this 3-byte/2-led schema. */
    int j = st_idx;
    for (int i = st_led; i <= stp_led; i += 2) {
        led_data_output[j] = led_buffer[i] >> 4;
        led_data_output[j+1] = (led_buffer[i] << 4) | (led_buffer[i + 1] >> 8);
        led_data_output[j+2] = led_buffer[i + 1] & 0xFF;
        j += 3;
    }
}

static void write_leds(void) 
{
    if (spi_transceive_cb(spi_dev, &spi_cfg, &tx, NULL, dma_spi_tx_callback, NULL) != 0) {
        LOG_ERR("Could not write to SPI device");
        return;
    }
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
 *                     |* ->run            |
 *                     |all other signals  |
 *                     |not allowed; assert|
 *
 *  State run:
 *
 *  |entry             |  |run             |  |exit           |
 *  |------------------|  |----------------|  |---------------|
 *  |* start conversion|  |led def write:  |  |not implemented|
 *  |  timer           |  |* set armed val |

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
    struct LED_Driver_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct LED_Driver_SM_Evt * p_evt = &p_inst->sm_evt;

    /* Expecting only an "init instance" event. Anything else is an error. */
    assert(p_evt->sig == k_LED_Driver_SM_Evt_Sig_Init_Instance);

    /* We init'd required params on the caller's thread (i.e.
     * LED_Driver_Init_Instance()), now finish the job. Since this is an
     * Init_Instance event the data contains the intance cfg. */
    struct LED_Driver_SM_Evt_Sig_Init_Instance * p_ii = &p_evt->data.init_inst;
    config_instance_deferred(p_inst, &p_ii->cfg);
    broadcast_instance_initialized(p_inst, p_ii->cfg.cb);

    led_driver_init(p_inst);

    // led_startup_animation(1000);

    smf_set_state(SMF_CTX(p_sm), &states[run]);
}

/* Run state responsibility is to be the root state to handle everything else
 * other than instance initialization. */

static void state_run_entry(void * o)
{
    struct smf_ctx * p_sm = o;
    struct LED_Driver_Instance * p_inst = sm_ctx_to_instance(p_sm);
}

static void state_run_run(void * o)
{
    struct smf_ctx * p_sm = o;
    struct LED_Driver_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct LED_Driver_SM_Evt * p_evt = &p_inst->sm_evt;

    switch(p_evt->sig){
        default: break;
        case k_LED_Driver_SM_Evt_Sig_Init_Instance:
            /* Should never occur. */
            assert(false);
            break;
        case k_LED_Driver_SM_Evt_Change_Default_Levels:
            uint8_t btn_id = p_evt->data.def_lvl.btn_id;
            uint8_t offset = p_evt->data.def_lvl.offset;
            bool arm_chk = p_evt->data.def_lvl.armed;
            uint16_t val = arm_chk ? DIM_ARMED_STEP : 0;
            uint8_t active = p_evt->data.def_lvl.step;
            enum LED_STATE arm = arm_chk ? LED_ARM : LED_DISARM;

            /* Assign button to channel and led. if button is greater than offset, it is channel 1 */
            int channel = 0; 
            if (btn_id >= offset) {
                channel = 1;
                btn_id -= offset;
            }

            update_leds(channel, btn_id, offset, val, arm, active);

             break;

        case k_LED_Driver_SM_Evt_LED_Driver_Reset_LED:
            
            update_leds(p_evt->data.reset.channel, p_evt->data.reset.step, p_evt->data.reset.offset, p_evt->data.reset.val, LED_STEP, NULL);
            break;

        #if CONFIG_FKMG_LED_DRIVER_SHUTDOWN_ENABLED
        case k_LED_Driver_Evt_Sig_Instance_Deinitialized:
            assert(false);
            break;
        #endif
    }
}


static const struct smf_state states[] = {
    /*                                      entry               run  exit */
    [  init] = SMF_CREATE_STATE(           NULL,   state_init_run, NULL),
    [   run] = SMF_CREATE_STATE(state_run_entry,    state_run_run, NULL),
    #if CONFIG_FKMG_LED_DRIVER_SHUTDOWN_ENABLED
    [deinit] = SMF_CREATE_STATE(           NULL, state_deinit_run, NULL),
    #endif
};

/* ******
 * Thread
 * ******/

static void thread(void * p_1, /* struct LED_Driver_Instance* */
        void * p_2_unused, void * p_3_unused)
{
    struct LED_Driver_Instance * p_inst = p_1;
    /* NOTE: smf_set_initial() executes the entry state. */
    struct smf_ctx * p_sm = &p_inst->sm;
    smf_set_initial(SMF_CTX(p_sm), &states[init]);

    /* Get the state machine event queue and point to where to put the dequeued
     * event. */
    struct k_msgq * p_msgq = p_inst->msgq.p_sm_evts;
    struct LED_Driver_SM_Evt * p_evt = &p_inst->sm_evt;

    bool run = true;

    while(run){
        /* Wait on state machine event. Cache it then run state machine. */
        k_msgq_get(p_msgq, p_evt, K_FOREVER);
        run = smf_run_state(SMF_CTX(p_sm)) == 0;
    }

}

static void start_thread(
        struct LED_Driver_Instance     * p_inst,
        struct LED_Driver_Instance_Cfg * p_cfg)
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


void clear_leds() {
    memset(led_buffer, 0, sizeof(led_buffer));
    pack_and_write_led_data();
}


void set_led_full_brightness(int led_number) {
    clear_leds();
    led_buffer[led_number] = QUARTER_BRIGHTNESS;
    pack_and_write_led_data();
}


void set_initial_armed_leds(void) {
    for (int i = 0; i < 16; i++) {
        led_armed[i] = true;
    }
}

void update_leds(bool channel, 
                 uint8_t step, 
                 uint8_t offset, 
                 uint16_t value,
                 enum LED_STATE state,
                 uint8_t active)
{
    if (state != LED_STEP) {
        buffer_armed_steps_leds(channel, step, offset, state);
        led_buffer[ led_to_out_map[(channel * offset) + active] ] = EIGHTH_BRIGHTNESS;  
    } else {
        buffer_armed_steps_leds(channel, step, offset, state);
        led_buffer[ led_to_out_map[(channel * offset) + step] ] = value;    
    }

    /* Map the LED number to the output channel. Channel is a bool value as there are only two possibilities here. If we are on channel 1 we need to take into account the step offset (i.e. where the second channel begins). Once we know which LED corresponds to the step we're trying to light up, we can redefine our index properly. Update value given in argument */

    /* Since the LED Driver maps 12-bit values into 8-bit packages, we need to write two LED channels over 3 bytes (24 bits total). These values are set to account this property of the driver and keeps track of where channel 0 and channel 1 begin and end in this 24 byte schema. */
    int byte_offset = (offset * 3) / 2;
    int led_offset = (offset % 2 == 0) ? byte_offset * 2 / 3 : byte_offset * 2 / 3 + 1;
    int start_idx, end_idx, start_led, end_led;

    /* Set our starting and ending indices so that we essentially decouple both channels. When we play/pause a channel, its last step is still visible. Without these we are constantly resetting all LEDs which isn't necessary. */
    if (channel == 1) {
        start_idx = 0;
        end_idx = byte_offset - 1;
        start_led = 0;
        end_led = led_offset - 1;
    } else {
        // channel 0
        start_idx = byte_offset;
        end_idx = 23;
        start_led = led_offset;
        end_led = 15;
    }
    
    map_led_buffer_to_24_bytes(start_idx, start_led, end_led);

    write_leds();

}


void LED_Driver_Init_Instance(struct LED_Driver_Instance_Cfg * p_cfg)
{
    /* Get pointer to instance to configure. */
    struct LED_Driver_Instance * p_inst = p_cfg->p_inst;

    init_module();
    init_instance(p_inst);
    config_instance_immediate(p_inst, p_cfg);
    add_instance_to_instances(p_inst);
    start_thread(p_inst, p_cfg);
    q_init_instance_event(p_cfg);
}


void LED_Driver_Add_Listener(struct LED_Driver_Listener_Cfg * p_cfg)
{
    struct LED_Driver_Listener * p_lsnr = p_cfg->p_lsnr;
    init_listener(p_lsnr);
    config_listener(p_lsnr, p_cfg);
    add_listener_for_signal_to_listener_list(p_cfg);
}


#if CONFIG_FKMG_LED_DRIVER_NO_OPTIMIZATIONS
#pragma GCC pop_options
#endif
