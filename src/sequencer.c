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

#include "led.h"
#include "led/evt.h"

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
#define TEST_DELAY_TIME         120

#define GPIO_PINS           DT_PATH(zephyr_user)

#define LOW_EDGE            0
#define HIGH_EDGE           1

#define GATE_HIGH           1
#define GATE_LOW            0

/* LED Brightness */
#define FULL_BRIGHTNESS     0xFFF
#define HALF_BRIGHTNESS     0x7FF
#define QUARTER_BRIGHTNESS  0x3FF
#define EIGHTH_BRIGHTNESS   0x1FF
#define DIM_ARMED_STEP      0x4B

#define LED_STEP            2



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
static uint32_t counter_freq; 


/* *****************************************************************************
 * Device Structs
 */

static const struct gpio_dt_spec gates[NUMBER_OF_GATES] = {
    GPIO_DT_SPEC_GET(GATE_PINS, gate_high_gpios), // Gate 0
    GPIO_DT_SPEC_GET(GATE_PINS, gate_low_gpios)   // Gate 1
};

static const struct gpio_dt_spec play_pause_a       = GPIO_DT_SPEC_GET(GPIO_PINS, pp_trigger_high_gpios); 
static const struct gpio_dt_spec play_pause_b       = GPIO_DT_SPEC_GET(TRANSPORT_PINS, pp_trigger_low_gpios); 
static const struct gpio_dt_spec reset_a            = GPIO_DT_SPEC_GET(TRANSPORT_PINS, rst_trigger_high_gpios);
static const struct gpio_dt_spec reset_b            = GPIO_DT_SPEC_GET(TRANSPORT_PINS, rst_trigger_low_gpios); 
static const struct gpio_dt_spec mode_btn           = GPIO_DT_SPEC_GET(GPIO_PINS, mode_global_gpios);

static const struct gpio_dt_spec play_led_high      = GPIO_DT_SPEC_GET(GPIO_PINS, led_17_gpios);
static const struct gpio_dt_spec reset_led_high     = GPIO_DT_SPEC_GET(GPIO_PINS, led_18_gpios);
static const struct gpio_dt_spec play_led_low       = GPIO_DT_SPEC_GET(GPIO_PINS, led_19_gpios);
static const struct gpio_dt_spec reset_led_low      = GPIO_DT_SPEC_GET(GPIO_PINS, led_20_gpios);
static const struct gpio_dt_spec mode_led           = GPIO_DT_SPEC_GET(GPIO_PINS, led_21_gpios);

static void on_transport_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_transport_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_reset_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
static void on_reset_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins); 


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

/*********************************************************************
 * NVS Storage
 * Configure NVS submodule to store sequencer instance and mode data
 ********************************************************************/


enum Sequencer_Nvs_Key {
    SEQ_ZERO,
    SEQ_ARMED,
    SEQ_OFFSET,
    SEQ_MAX_STEP,
    SEQ_MODE
};

static struct nvs_fs fs;

#define NVS_PARTITION		    storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

static void nvs_init(void)
{
    int rc = 0, cnt = 0, cnt_his = 0;
    
    struct flash_pages_info info;
    
    /* define the nvs file system by settings with: sector_size equal to the pagesize, 3 sectors starting at NVS_PARTITION_OFFSET */
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device %s is not ready\n", fs.flash_device->name);
        return;
    }
    
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        LOG_ERR("Unable to get page info\n");
        return;
    }
    fs.sector_size = info.size;
    fs.sector_count = 3U;
    
    rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("Flash Init failed\n");
        return;
    }
}

static void fetch_nvs_data(struct Sequencer_Instance * p_inst)
{
    int rc = 0; 
    bool active[16];
    uint8_t offset;
    uint8_t max_step[2];
    uint8_t mode;
    
    rc = nvs_read(&fs, SEQ_ARMED, &active, sizeof(active));
    if (rc < 0) {
        LOG_ERR("No data found in NVS for armed sequencers. Setting all to true.");
        for (int i = 0; i < 16; i++) {
            p_inst->seq.active[i] = true;
        }
    } else {
        for (int i = 0; i < 16; i++) {
            p_inst->seq.active[i] = active[i];
        }
    }
    
    rc = nvs_read(&fs, SEQ_OFFSET, &offset, sizeof(offset));
    if (rc < 0) {
        LOG_ERR("No data found in NVS for offset. Setting to 8.");
        p_inst->seq.offset = 8;
    } else {
        p_inst->seq.offset = offset; 
    }
    
    rc = nvs_read(&fs, SEQ_MAX_STEP, &max_step, sizeof(max_step));
    if (rc < 0) {
        LOG_ERR("No data found in NVS for max step. Setting to 8.");
        for (int i = 0; i < 2; i++) {
            p_inst->seq.maxStep[i] = 8;
        }
    } else {
        for (int i = 0; i < 2; i++) {
            p_inst->seq.maxStep[i] = max_step[i];
        }
    }
    
    rc = nvs_read(&fs, SEQ_MODE, &mode, sizeof(mode));
    if (rc < 0) {
        LOG_ERR("No data found in NVS for mode. Setting to 0.");
        p_inst->seq.mode = 0;
    } else {
        p_inst->seq.mode = mode; 
    }
}


static int write_nvs_data(struct Sequencer_Instance * p_inst, 
                            enum Sequencer_Nvs_Key key,
                            void * data)
{
    int rc = 0; 
    rc = nvs_write(&fs, key, &data, sizeof(data));
    if (rc < 0) {
        LOG_ERR("Error: Failed to write %d value to NVS", key);
    }
}


/*****************
 * Timer Functions
/*****************/


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
 * The on-time is calculated as half the time delay between when this rising edge hits and the next gate. The delay_buffer variable in the instance is to keep track of this delay so we only have to calculate it once, unless one of an active channel's pots sends a pot_change event, in which case we will recalculate the new delay time if it's below a certain threshold, that is, if there's enough time left in the current step to make it worth recalculating.
*/ 

static uint16_t calculate_gate_timer_delay(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, bool edge) 
{
    uint16_t delay_val; 

    if (edge) {
        delay_val = p_inst->seq.time[next_step(p_inst, id)]; 
        delay_val = delay_val >> 1; 
        p_inst->seq.delay_buffer[id] = delay_val; 
        return delay_val; 
    } 
    
    return p_inst->seq.delay_buffer[id]; 
}

/* **************
 * Listener Utils
 * **************/

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



/* ************************
 * Hardware Timer Inits
 * ************************/

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

    return; 

}


/* **************
 * GPIO Configs
 * *************/

static void transport_gpio_pin_config(void) {
    int ret;
    
    ret = gpio_pin_configure(play_pause_a.port, play_pause_a.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure playPauseA pin");
    }

    ret = gpio_pin_configure(play_pause_b.port, play_pause_b.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure playPauseB pin");
    }

    ret = gpio_pin_configure(reset_a.port, reset_a.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure resetA pin");
    }

    ret = gpio_pin_configure(reset_b.port, reset_b.pin, GPIO_INPUT | GPIO_PULL_DOWN) ;
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure resetB pin");
    }

    if  (   !device_is_ready(play_led_high.port)
        ||  !device_is_ready(play_led_low.port)
        ||  !device_is_ready(reset_led_high.port)
        ||  !device_is_ready(reset_led_low.port)
        ||  !device_is_ready(mode_led.port)
        )
    {
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

    ret = gpio_pin_configure_dt(&mode_led, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure Play Pause LED pin, %d", ret);
        return;
    }
}


static void transport_gpio_interrupt_config(void)
{
    int ret;

    ret = gpio_pin_interrupt_configure(play_pause_a.port, play_pause_a.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on playPauseA pin");
    }

    ret = gpio_pin_interrupt_configure(play_pause_b.port, play_pause_b.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on playPauseB pin");
    }

    ret = gpio_pin_interrupt_configure(reset_a.port, reset_a.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on resetA pin");
    }

    ret = gpio_pin_interrupt_configure(reset_b.port, reset_b.pin, GPIO_INT_EDGE_BOTH);
    if (ret < 0) {
        LOG_ERR("Error: Failed to configure interrupt on resetB pin");
    }
}


static void transport_gpio_callback_config(struct Sequencer_Instance * p_inst)
{
    int ret; 

    gpio_init_callback(&p_inst->transport_gpio_1_cb, on_transport_a, BIT(play_pause_a.pin));
    gpio_init_callback(&p_inst->transport_gpio_2_cb, on_transport_b, BIT(play_pause_b.pin));
    gpio_init_callback(&p_inst->transport_gpio_1_rst_cb, on_reset_a, BIT(reset_a.pin));
    gpio_init_callback(&p_inst->transport_gpio_2_rst_cb, on_reset_b, BIT(reset_b.pin));

    ret = gpio_add_callback(play_pause_a.port, &p_inst->transport_gpio_1_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on playPauseA pin");
    }

    ret = gpio_add_callback(play_pause_b.port, &p_inst->transport_gpio_2_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on playPauseB pin");
    }

    ret = gpio_add_callback(reset_a.port, &p_inst->transport_gpio_1_rst_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on resetA pin");
    }

    ret = gpio_add_callback(reset_b.port, &p_inst->transport_gpio_2_rst_cb);
    if (ret < 0) {
        LOG_ERR("Error: Failed to add callback on resetB pin");
    }
}


static void transport_gpio_init(struct Sequencer_Instance * p_inst) {
    
    int ret = 0; 

    if (!device_is_ready(play_pause_a.port) ||
        !device_is_ready(play_pause_b.port) ||
        !device_is_ready(reset_a.port) ||
        !device_is_ready(reset_b.port)) 
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


/* ************************
 * Initial Value Configs
 * ************************/

/* FIXME: Testing Only. Replace with an initial ADC read routine*/

static void config_initial_voltages(struct Sequencer_Instance * p_inst)
{
    for (int i = 0; i < ARRAY_SIZE(p_inst->seq.voltage); ++i)
    {
        p_inst->seq.voltage[i] = 0 + (i * 500);
    }
}


static void config_initial_time_delays(struct Sequencer_Instance * p_inst)
{
    for (int i = 0; i < ARRAY_SIZE(p_inst->seq.time); ++i)
    {
        p_inst->seq.time[i] = TEST_DELAY_TIME;
    }
}


static void config_initial_sequencer_values(struct Sequencer_Instance * p_inst)
{
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



/* ***********************
 * Timer Callback Inits
 * ***********************/

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


/* ********************************
 * Potentiometer Update Functions
 * ********************************/

static void post_updated_pot_value(struct Sequencer_Instance * p_inst, enum Pot_Id id, uint16_t val) 
{
    if (id < 16) {
        p_inst->seq.voltage[id] = val; 
    } else if (id > 16 && id < 32) {
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

/* ********************
 * Stepping Functions
 * ********************/

static void set_voltage_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    uint8_t channel = id + 1;

    dac_write_new_value(channel, p_inst->seq.voltage[p_inst->seq.step[id]]); 

}

static void set_gate_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, bool status) 
{
    if (id < 0 || id >= NUMBER_OF_GATES) return;

    int result = gpio_pin_set_dt(&gates[id], status);

    if (result < 0) LOG_ERR("Gate %d Error: %d\n", id, result);
}


static void advance_sequencer_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    if (p_inst->seq.maxStep[id] == 0) return;

    uint8_t next_step = (p_inst->seq.step[id] + 1) % p_inst->seq.maxStep[id];

    p_inst->seq.step[id] = next_step; 
}


/* *******************
 * Transport Buttons
 * *******************/


enum Sequencer_Ctrl_Id {
    PLAY_PAUSE_A,
    RESET_A,
    PLAY_PAUSE_B,
    RESET_B,
    MODE_BTN,
    CTRL_BTN_LAST
};


struct ButtonDefinition {
    const struct device *port;
    gpio_pin_t pin;
};

const struct ButtonDefinition button_definition[5] = {
    { .port = play_pause_a.port, .pin = play_pause_a.pin },
    { .port = reset_a.port, .pin = reset_a.pin },
    { .port = play_pause_b.port, .pin = play_pause_b.pin },
    { .port = reset_b.port, .pin = reset_b.pin },
    { .port = mode_btn.port, .pin = mode_btn.pin }
};


static int debounce (enum Sequencer_Ctrl_Id id) {
    static uint8_t switch_state[CTRL_BTN_LAST] = {0}; 
    switch_state[id] = (switch_state[id] << 1) | !gpio_pin_get(button_definition[id].port, button_definition[id].pin);
    if (switch_state[id] == 0x00) {
        switch_state[id] = 0xFF;
        return 1; 
    }
}

/* FIXME: move as much out of ISRs */
static void on_transport_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance *p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_1_cb);

    static bool status = false;
    bool pressed = debounce(PLAY_PAUSE_A);

    if (pressed){
        status = !status;
        gpio_pin_set(play_led_high.port, play_led_high.pin, status);

        if (p_inst->seq.running[0]) {
            counter_stop(p_inst->timer.t[0]);
        } else {
            counter_start(p_inst->timer.t[0]);
        }
        p_inst->seq.running[0] = !p_inst->seq.running[0];
    }
}
 

static void on_transport_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance *p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_2_cb);

    static bool status = false;
    bool pressed = debounce(PLAY_PAUSE_B);

    if (pressed){
        status = !status;
        gpio_pin_set(play_led_low.port, play_led_low.pin, status);

        if (p_inst->seq.running[1]) {
            counter_stop(p_inst->timer.t[1]);
        } else {
            counter_start(p_inst->timer.t[1]);
        }
        p_inst->seq.running[1] = !p_inst->seq.running[1];
    }
}

static void on_reset_a(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance * p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_1_rst_cb);
    bool pressed = debounce(RESET_A);

    if (pressed){
        gpio_pin_set(reset_led_high.port, reset_led_high.pin, 1);
        p_inst->seq.step[0] = 0;
        p_inst->seq.edge[0] = false;
    }

    struct Sequencer_Evt evt = {
        .sig = k_Seq_Evt_Sig_Reset_LED,
        .data.reset.channel = 0,
        .data.reset.offset = p_inst->seq.offset,
        .data.reset.step = p_inst->seq.step[0],
        .data.reset.val = EIGHTH_BRIGHTNESS
    };

    broadcast_event_to_listeners(p_inst, &evt);

    bool btn_state = gpio_pin_get(reset_a.port, reset_a.pin);
    gpio_pin_set(reset_led_high.port, reset_led_high.pin, btn_state);
}


static void on_reset_b(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Sequencer_Instance * p_inst = CONTAINER_OF(cb, struct Sequencer_Instance, transport_gpio_2_rst_cb);
    bool pressed = debounce(RESET_B);

    if (pressed){
        gpio_pin_set(reset_led_low.port, reset_led_low.pin, 1);
        p_inst->seq.step[1] = 0;
        p_inst->seq.edge[1] = false;
    }

    struct Sequencer_Evt evt = {
        .sig = k_Seq_Evt_Sig_Reset_LED,
        .data.reset.channel = 1,
        .data.reset.offset = p_inst->seq.offset,
        .data.reset.step = p_inst->seq.step[1],
        .data.reset.val = EIGHTH_BRIGHTNESS
    };

    broadcast_event_to_listeners(p_inst, &evt);

    bool btn_state = gpio_pin_get(reset_b.port, reset_b.pin);
    gpio_pin_set(reset_led_low.port, reset_led_low.pin, btn_state);
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
 *  |* start conversion|  |run sequencer:  |  |not implemented|
 *  |  timer           |  |* timer elapsed |
 *                        |* set voltage   |
 *                        |* set gate      |
 *                        |* adv step      |
 *                        |* set LED       |
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
            uint32_t new_tix; // changed from uint16
            enum Sequencer_Id id = p_stepped->id;

            switch(p_stepped->edge) {

                case LOW_EDGE:

                    uint8_t currentStep = p_inst->seq.step[id];
                    bool isActive = p_inst->seq.active[currentStep + id * p_inst->seq.offset];

                    update_leds(id, currentStep, p_inst->seq.offset, EIGHTH_BRIGHTNESS, LED_STEP, NULL);
                    
                    if (isActive) {
                        set_voltage_on_step(p_inst, id);
                        set_gate_on_step(p_inst, id, true);
                    }

                    p_inst->seq.edge[id] = HIGH_EDGE;
                    
                    new_tix = calculate_gate_timer_delay(p_inst, id, HIGH_EDGE);
                    
                    if (new_tix < 1) new_tix = 1;
                    
                    reset_timer(p_inst, id, new_tix);
                    
                break;

                case HIGH_EDGE:

                    advance_sequencer_step(p_inst, id);
                    set_gate_on_step(p_inst, id, false);
                    
                    p_inst->seq.edge[id] = LOW_EDGE;
                    
                    new_tix = calculate_gate_timer_delay(p_inst, id, LOW_EDGE);
                    
                    if (new_tix < 1) new_tix = 1;
                    
                    reset_timer(p_inst, id, new_tix);
                break; 
            }
            break;

        case k_Seq_SM_Evt_Sig_Pot_Value_Changed:
            struct Sequencer_SM_Evt_Sig_Pot_Value_Changed * p_pot_changed = &p_evt->data.pot_changed; 
            post_updated_pot_value(p_inst, p_pot_changed->pot_id, p_pot_changed->val);
            // check_current_step(p_inst, p_pot_changed->pot_id);

            break;
        case k_Seq_SM_Evt_Sig_Button_Pressed:


            break;
    }
}


static const struct smf_state states[] = {
    /*                                      entry               run  exit */
    [  init] = SMF_CREATE_STATE(           NULL,   state_init_run, NULL),
    [   run] = SMF_CREATE_STATE(state_run_entry,    state_run_run, NULL),
};

/* ******
 * Thread
 * ******/

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



void Sequencer_Init_Instance(struct Sequencer_Instance_Cfg * p_cfg)
{
    /* Get pointer to instance to configure. */
    struct Sequencer_Instance * p_inst = p_cfg->p_inst;

    init_module();
    init_instance(p_inst);
    config_instance_immediate(p_inst, p_cfg);
    add_instance_to_instances(p_inst);
    start_thread(p_inst, p_cfg);
    q_init_instance_event(p_cfg);
}


void Sequencer_Add_Listener(struct Sequencer_Listener_Cfg * p_cfg)
{
    struct Sequencer_Listener * p_lsnr = p_cfg->p_lsnr;
    init_listener(p_lsnr);
    config_listener(p_lsnr, p_cfg);
    add_listener_for_signal_to_listener_list(p_cfg);
}


#if CONFIG_FKMG_SEQ_NO_OPTIMIZATIONS
#pragma GCC pop_options
#endif
