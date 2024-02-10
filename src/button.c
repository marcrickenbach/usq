/* *****************************************************************************
 * @brief Button implementation.
 */

/* *****************************************************************************
 * TODO
 */

/* *****************************************************************************
 * Includes
 */

#include "button.h"

#include "button/private/sm_evt.h"
#include "button/private/module_data.h"

#include <zephyr/smf.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dma.h>

#include <assert.h>


/* *****************************************************************************
 * Constants, Defines, and Macros
 */

#define SUCCESS (0)
#define FAIL    (-1)

#define OVERRIDE            true
#define NO_OVERRIDE         false

#define BUTTON_PINS         DT_PATH(zephyr_user)
#define I2C2_NODE           DT_NODELABEL(i2c2)

#define DEBOUNCE_DELAY      10

#define MCP23017_ADDR       0x20

#define GPIO_PINS           DT_PATH(zephyr_user)


/* *****************************************************************************
 * Debugging
 */

#if CONFIG_FKMG_BUTTON_NO_OPTIMIZATIONS
#pragma GCC push_options
#pragma GCC optimize ("Og")
#warning "button.c unoptimized!"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(button);

/* *****************************************************************************
 * Structs
 */

static struct button_module_data button_md = {0};
/* Convenience accessor to keep name short: md - module data. */
#define md button_md

const struct device * i2c_dev = DEVICE_DT_GET(I2C2_NODE);

// const struct gpio_dt_spec button_input  = GPIO_DT_SPEC_GET(BUTTON_PINS, sw_input_gpios); 
const struct gpio_dt_spec sw_input_int  = GPIO_DT_SPEC_GET(GPIO_PINS, sw_input_gpios);
const struct gpio_dt_spec sw_address_0  = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_0_gpios);
const struct gpio_dt_spec sw_address_1  = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_1_gpios);
const struct gpio_dt_spec sw_address_2  = GPIO_DT_SPEC_GET(GPIO_PINS, sw_addr_2_gpios);


/* *****************************************************************************
 * Private
 */

/* *****
 * Utils
 * *****/

static bool instance_contains_state_machine_ctx(
        struct Button_Instance * p_inst,
        struct smf_ctx      * p_sm_ctx)
{
    return(&p_inst->sm == p_sm_ctx);
}

/* Find the instance that contains the state machine context. */
static struct Button_Instance * sm_ctx_to_instance(struct smf_ctx * p_sm_ctx)
{
    /* Iterate thru the instances. */
    struct Button_Instance * p_inst = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&md.list.instances, p_inst, node.instance){
        if(instance_contains_state_machine_ctx(p_inst, p_sm_ctx)) return p_inst;
    }
    return(NULL);
}


/* **************
 * Listener Utils
 * **************/

static void clear_listener(struct Button_Listener * p_lsnr)
{
    memset(p_lsnr, 0, sizeof(*p_lsnr));
}

static void config_listener(
        struct Button_Listener     * p_lsnr,
        struct Button_Listener_Cfg * p_cfg)
{
    /* Set listner's instance it is listening to. */
    p_lsnr->p_inst = p_cfg->p_inst;

    /* Set listner's callback. */ 
    p_lsnr->cb = p_cfg->cb;
}

static void init_listener(struct Button_Listener * p_lsnr)
{
    clear_listener(p_lsnr);
}

/* **************
 * Instance Utils
 * **************/

static void add_instance_to_instances(
        struct Button_Instance  * p_inst)
{
    sys_slist_append(&md.list.instances, &p_inst->node.instance);
}

static void config_instance_queues(
        struct Button_Instance     * p_inst,
        struct Button_Instance_Cfg * p_cfg)
{
    p_inst->msgq.p_sm_evts = p_cfg->msgq.p_sm_evts;
}

/* Forward reference */

static void on_button_press(const struct device *dev, struct gpio_callback *cb, uint32_t pins);
 
/* Configure Button Interrupt and Callback */
static void button_interrupt_init(struct Button_Instance * p_inst) 
{
    
    int ret = gpio_pin_configure_dt(&sw_input_int, GPIO_INPUT | GPIO_PULL_UP);
    assert(ret == 0); 

    ret = gpio_pin_interrupt_configure_dt(&sw_input_int, GPIO_INT_EDGE_FALLING);
    assert(ret == 0); 

    gpio_init_callback(&p_inst->button_pressed_gpio_cb, on_button_press, BIT(sw_input_int.pin));

    ret = gpio_add_callback(sw_input_int.port, &p_inst->button_pressed_gpio_cb);
    assert(ret == 0);

}


static void button_gpio_init(struct Button_Instance * p_inst) {


    if  (   !device_is_ready(sw_input_int.port)
         ||   !device_is_ready(sw_address_0.port)
         ||   !device_is_ready(sw_address_1.port)
         ||   !device_is_ready(sw_address_2.port)
        ){
        LOG_ERR("GPIO Ready: Failed");
        return; 
    }

    button_interrupt_init(p_inst);

    int ret = gpio_pin_configure_dt(&sw_address_0, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure LED latch enable pin, %d", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&sw_address_1, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure LED latch enable pin, %d", ret);
        return;
    }

    ret = gpio_pin_configure_dt(&sw_address_2, GPIO_OUTPUT_LOW);
    if (ret < 0) {
        LOG_ERR("Could not configure LED latch enable pin, %d", ret);
        return;
    }

}


static void config_instance_deferred(
        struct Button_Instance     * p_inst,
        struct Button_Instance_Cfg * p_cfg)
{

}

/* Since configuration starts on caller's thread, configure fields that require
 * immediate and/or inconsequential configuration and defer rest to be handled
 * by our own thread later. */
static void config_instance_immediate(
        struct Button_Instance     * p_inst,
        struct Button_Instance_Cfg * p_cfg)
{
    config_instance_queues(p_inst, p_cfg);
}

static void init_instance_lists(struct Button_Instance * p_inst)
{
    for(enum Button_Evt_Sig sig = k_Button_Evt_Sig_Beg;
                         sig < k_Button_Evt_Sig_End;
                         sig++){
        sys_slist_init(&p_inst->list.listeners[sig]);
    }
}

static void clear_instance(struct Button_Instance * p_inst)
{
    memset(p_inst, 0, sizeof(*p_inst));
}

static void init_instance(struct Button_Instance * p_inst)
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
        struct Button_Evt      * p_evt,
        struct Button_Listener * p_lsnr)
{
    /* call the listener, passing the event */
    if(p_lsnr->cb) p_lsnr->cb(p_evt);
}

static void broadcast_event_to_listeners(
        struct Button_Instance * p_inst,
        struct Button_Evt      * p_evt)
{
    enum Button_Evt_Sig sig = p_evt->sig;
    struct Button_Listener * p_lsnr = NULL;
    SYS_SLIST_FOR_EACH_CONTAINER(&p_inst->list.listeners[sig], p_lsnr, node.listener){
        broadcast(p_evt, p_lsnr);
    }
}

/* There's only 1 listener for instance initialization, and it is provided in
 * the cfg struct. */
static void broadcast_instance_initialized(
        struct Button_Instance * p_inst,
        Button_Listener_Cb       cb)
{
    struct Button_Evt evt = {
            .sig = k_Button_Evt_Sig_Instance_Initialized,
            .data.initd.p_inst = p_inst
    };

    struct Button_Listener lsnr = {
        .p_inst = p_inst,
        .cb = cb
    };

    broadcast(&evt, &lsnr);
}



/* **************
 * Listener Utils
 * **************/

static void add_listener_for_signal_to_listener_list(
    struct Button_Listener_Cfg * p_cfg)
{
    /* Get pointer to interface to add listener to. */
    struct Button_Instance * p_inst = p_cfg->p_inst;

    /* Get pointer to configured listener. */
    struct Button_Listener * p_lsnr = p_cfg->p_lsnr;

    /* Get signal to listen for. */
    enum Button_Evt_Sig sig = p_cfg->sig;

    /* Add listener to instance's specified signal. */
    sys_slist_t * p_list = &p_inst->list.listeners[sig];
    sys_snode_t * p_node = &p_lsnr->node.listener;
    sys_slist_append(p_list, p_node);
}

static bool signal_has_listeners(
        struct Button_Instance * p_inst,
        enum Button_Evt_Sig      sig)
{
    return(!sys_slist_is_empty(&p_inst->list.listeners[sig]));
}

/* **************
 * Event Queueing
 * **************/

static void q_sm_event(struct Button_Instance * p_inst, struct Button_SM_Evt * p_evt)
{
    bool queued = k_msgq_put(p_inst->msgq.p_sm_evts, p_evt, K_NO_WAIT) == 0;

    if(!queued) assert(false);
}

static void q_init_instance_event(struct Button_Instance_Cfg * p_cfg)
{
    struct Button_SM_Evt evt = {
            .sig = k_Button_SM_Evt_Sig_Init_Instance,
            .data.init_inst.cfg = *p_cfg
    };
    struct Button_Instance * p_inst = p_cfg->p_inst;
    q_sm_event(p_inst, &evt);
}

static void q_btn_pressed_event(struct Button_Instance * p_inst, struct Button_SM_Evt * p_evt)
{
    struct Button_SM_Evt evt = {
            .sig = k_Button_SM_Evt_Sig_Pressed,
            .data.pressed.btn_state[0] = p_evt->data.pressed.btn_state[0],
            .data.pressed.btn_state[1] = p_evt->data.pressed.btn_state[1]
    };
    q_sm_event(p_inst, &evt);
}


/***********************************************
*   MCP23017 I2C Functions
************************************************/

/* This function was scheduled by our press button interrupt callback. First we clear our last read buffers, then we flush the interrupt registers so that we disable interrupts while we process our data. When we read from the interrupt capture and gpio a/b registers, we both receive the information we need and clear the interrupt flags, setting the signal high again to wait for the next interrupt. We then schedule an event with the new reading data. Read from INTFA and INTFB to fetch interrupt flag, as well as the INTCAPA/B regs to clear interrupt flags. Doing it in this way allows us to check the flag registers against the capture registers, which hold on to the previous data. This will allow us to distinguish between multiple button pushes on the same button simply by comparing corresponding elements -- those with the same iformation designate a push. */

static const uint8_t btn_map_port[16] = {   4, 5, 6, 7, 15, 14, 13, 12, 
                                            11, 10, 9, 8, 0, 1, 2, 3}; 


static int debounce (void) {
    static uint8_t switch_state = 0; 
    switch_state = (switch_state << 1) | !gpio_pin_get(sw_input_int.port, sw_input_int.pin);
    if (switch_state == 0x00) {
        switch_state = 0xFF;
        return 1; 
    }
}


/* GPIO Callback */
static void on_button_press(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct Button_Instance * p_inst = CONTAINER_OF(cb, struct Button_Instance, button_pressed_gpio_cb);

    /* Debounce */
    static uint64_t time_last = 0;
    uint64_t current_time = k_uptime_get();
    if (current_time - time_last < DEBOUNCE_DELAY) {
        // do we need to clear the interrupt, assuming it's still up after a false press? 
        return;
    }

    time_last = current_time;

        /* q_sm_event for k_Button_SM_Evt_Sig_Pressed */
        struct Button_SM_Evt evt = {
            .sig = k_Button_SM_Evt_Sig_Pressed,
        };

        q_sm_event(p_inst, &evt);
}


static int flush_i2c_interrupt_registers(void) {

    static const uint8_t flush_registers[2] = {0x00, 0x00};
    
    int ret = i2c_burst_write(i2c_dev, MCP23017_ADDR, MCP23017_GPINTENA, flush_registers, sizeof(flush_registers));
   
    if (ret < 0) {
        LOG_ERR("Could not write to MCP23017 registers, %d", ret);
    }

    return ret;
}


static void clear_previous_button_readings(struct Button_Instance * p_inst) 
{
    memset(p_inst->read_buf, 0, sizeof(p_inst->read_buf));
    memset(p_inst->btn_data, 0, sizeof(p_inst->btn_data));
}


static int button_read_data(struct Button_Instance * p_inst)
{       
    int ret = i2c_burst_read(i2c_dev, MCP23017_ADDR, MCP23017_INTCAPA, p_inst->read_buf, sizeof(p_inst->read_buf)); 
    if (ret < 0) {
        LOG_ERR("Could not read from MCP23017 registers, %d", ret);
    } 
    return ret; 
}


static int refill_i2c_interrupt_registers(void) {

    static const uint8_t refill_registers[2] = {0xFF, 0xFF};
    
    int ret = i2c_burst_write(i2c_dev, MCP23017_ADDR, MCP23017_GPINTENA, refill_registers, sizeof(refill_registers));
    
    if (ret < 0) {
        LOG_ERR("Could not write to MCP23017 registers, %d", ret);
    }
    
    return ret;
}


static uint8_t button_parse_readings(uint8_t * btn_readings) 
{
    int button = 0; 

    for (int i = 0; i < 2; i++) {

        if (btn_readings[i] == btn_readings[i+2]) {
            
            for (int k = 0; k <8; k++) {
                if (btn_readings[i] & (1 << k)) {
                    
                    // tells us which button was pressed
                    button = btn_map_port[k + (i * 8)]; 
                    return button;
                }
            }
        }
    }

    // if we get here, return an error code that cannot be a button number
    return -1;
}


static void button_after_press_work_handler(struct Button_Instance *p_inst) 
{
    uint8_t btn_readings[4];
    uint8_t temp = p_inst->btn_data[0]; 
    uint8_t btn_pressed;

    memcpy(btn_readings, p_inst->read_buf, sizeof(p_inst->read_buf));
    memcpy(btn_pressed, temp, sizeof(temp));

    // Return button value (x of 15)
    btn_pressed = button_parse_readings(&btn_readings);
    
    if (btn_pressed < 0) {
        LOG_ERR("Could not parse button readings, %d", btn_pressed); 
        return; 
    }

    struct Button_Evt evt = {
        .sig = k_Button_Evt_Sig_Pressed,
        .data.pressed.btn_id = btn_pressed,
    };

    broadcast_event_to_listeners(p_inst, &evt);
}




/* Helper Functions */
static uint8_t init_read_expander_register(uint8_t reg) 
{
    uint8_t buf[1] = {reg};
    uint8_t read_val[1] = {0};

    int ret = i2c_write_read(i2c_dev, MCP23017_ADDR, buf, 1, read_val, 1);
    assert(ret == 0);

    return read_val[0];
}


static int init_write_both_expander_registers(uint8_t reg, uint8_t val) 
{
    uint8_t reg_write[3] = {reg, val, val};

    int ret = i2c_write(i2c_dev, reg_write, sizeof(reg_write), MCP23017_ADDR);

}


/* Init Functions */

static void MCP23017_i2c_init(struct Button_Instance * p_inst) 
{
    uint8_t write_buf[2];
    uint8_t read_buf[1];

    int ret = i2c_dev == NULL || !device_is_ready(i2c_dev); 
    assert(ret == 0);   

    i2c_configure(i2c_dev, I2C_SPEED_FAST);
    assert(ret == 0);

    // mirror interrupts, disable sequential mode, open drain
    ret = init_write_both_expander_registers(MCP23017_IOCONA, 0b01000100);
    assert(ret == 0);

    // Set direction
    ret = init_write_both_expander_registers(MCP23017_IODIRA, 0x7F);
    assert(ret == 0);

    // enable pull-up on switches
    ret = init_write_both_expander_registers(MCP23017_GPPUA, 0xFF);
    assert(ret == 0);

    // invert polarity
    ret = init_write_both_expander_registers(MCP23017_IPOLA, 0xFF);
    // assert(ret == 0);

    // set interrupt-on-change
    ret = init_write_both_expander_registers(MCP23017_INTCONA, 0xFF);
    assert(ret == 0);

    // enable all interrupts
    ret = init_write_both_expander_registers(MCP23017_GPINTENA, 0xFF);
    assert(ret == 0);

    // Read from MCP23017 registers to clear them. 
    init_read_expander_register(MCP23017_INTCAPA);
    init_read_expander_register(MCP23017_INTCAPB);
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
 *  |entry             |  |run               |  |exit           |
 *  |------------------|  |------------------|  |---------------|
 *  |not implemented   |  |Button Press      |  |not implemented|
 *  |                  |  |* flush i2c regs  |
 *                        |* clear prev read |
 *                        |* read i2c data   |
 *                        |* process btn read|
 *                        |* refill i2c regs |
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
    run,    // Run - handles all events while running
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
    struct Button_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct Button_SM_Evt * p_evt = &p_inst->sm_evt;

    /* Expecting only an "init instance" event. Anything else is an error. */
    assert(p_evt->sig == k_Button_SM_Evt_Sig_Init_Instance);

    /* We init'd required params on the caller's thread (i.e.
     * Button_Init_Instance()), now finish the job. Since this is an
     * Init_Instance event the data contains the intance cfg. */
    struct Button_SM_Evt_Sig_Init_Instance * p_ii = &p_evt->data.init_inst;
    
    button_gpio_init(p_inst);

    MCP23017_i2c_init(p_inst);

    broadcast_instance_initialized(p_inst, p_ii->cfg.cb);
    smf_set_state(SMF_CTX(p_sm), &states[run]);
}

/* Run state responsibility is to be the root state to handle everything else
 * other than instance initialization. */

static void state_run_entry(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Button_Instance * p_inst = sm_ctx_to_instance(p_sm);

}

static void state_run_run(void * o)
{
    struct smf_ctx * p_sm = o;
    struct Button_Instance * p_inst = sm_ctx_to_instance(p_sm);

    /* Get the event. */
    struct Button_SM_Evt * p_evt = &p_inst->sm_evt;

    switch(p_evt->sig){
        default: break;

        case k_Button_SM_Evt_Sig_Init_Instance:
            /* Should never occur. */
            assert(false);
            break;

        case k_Button_SM_Evt_Sig_Pressed:

            flush_i2c_interrupt_registers();
            clear_previous_button_readings(p_inst);

            int ret = button_read_data(p_inst);
            if (ret < 0) {
                LOG_ERR("Could not read from MCP23017 registers, %d", ret);
            }

            button_after_press_work_handler(p_inst);

            struct Button_SM_Evt evt = {
                .sig = k_Button_SM_Evt_Sig_Reset_Interrupt,
            };
            q_sm_event(p_inst, &evt);

            /* FIXME: I'm not a fan of this delay. Switching out IC for SPI version so I'll have to redo this driver anyway, so if this works, keep it for now. */
            k_msleep(250);

            break;

        case k_Button_SM_Evt_Sig_Reset_Interrupt:
            
            ret = refill_i2c_interrupt_registers();
            if (ret < 0) {
                LOG_ERR("Could not write to MCP23017 registers, %d", ret);
            }

            break;
        #if CONFIG_FKMG_BUTTON_SHUTDOWN_ENABLED
        case k_Button_Evt_Sig_Instance_Deinitialized:
            assert(false);
            break;
        #endif
    }
}


static const struct smf_state states[] = {
    /*                                      entry               run  exit */
    [  init] = SMF_CREATE_STATE(           NULL,   state_init_run, NULL),
    [   run] = SMF_CREATE_STATE(state_run_entry,    state_run_run, NULL),
    #if CONFIG_FKMG_BUTTON_SHUTDOWN_ENABLED
    [deinit] = SMF_CREATE_STATE(           NULL, state_deinit_run, NULL),
    #endif
};

/* ******
 * Thread
 * ******/

static void thread(void * p_1, /* struct Button_Instance* */
        void * p_2_unused, void * p_3_unused)
{
    struct Button_Instance * p_inst = p_1;
    /* NOTE: smf_set_initial() executes the entry state. */
    struct smf_ctx * p_sm = &p_inst->sm;
    smf_set_initial(SMF_CTX(p_sm), &states[init]);

    /* Get the state machine event queue and point to where to put the dequeued
     * event. */
    struct k_msgq * p_msgq = p_inst->msgq.p_sm_evts;
    struct Button_SM_Evt * p_evt = &p_inst->sm_evt;

    bool run = true;

    while(run){
        /* Wait on state machine event. Cache it then run state machine. */
        k_msgq_get(p_msgq, p_evt, K_FOREVER);
        run = smf_run_state(SMF_CTX(p_sm)) == 0;
    }

}

static void start_thread(
        struct Button_Instance     * p_inst,
        struct Button_Instance_Cfg * p_cfg)
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

void Button_Init_Instance(struct Button_Instance_Cfg * p_cfg)
{
    /* Get pointer to instance to configure. */
    struct Button_Instance * p_inst = p_cfg->p_inst;

    init_module();
    init_instance(p_inst);
    config_instance_immediate(p_inst, p_cfg);
    add_instance_to_instances(p_inst);
    start_thread(p_inst, p_cfg);
    q_init_instance_event(p_cfg);
}


void Button_Add_Listener(struct Button_Listener_Cfg * p_cfg)
{
    struct Button_Listener * p_lsnr = p_cfg->p_lsnr;
    init_listener(p_lsnr);
    config_listener(p_lsnr, p_cfg);
    add_listener_for_signal_to_listener_list(p_cfg);
}


#if CONFIG_FKMG_BUTTON_NO_OPTIMIZATIONS
#pragma GCC pop_options
#endif
