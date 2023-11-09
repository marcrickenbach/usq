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

#define SUCCESS (0)
#define FAIL    (-1)

#define OVERRIDE    true
#define NO_OVERRIDE false

#define TIMER_X DT_INST(0, st_stm32_counter) 
#define TIMER_Y DT_INST(1, st_stm32_counter)

// Gate Outs
#define GATE_PINS DT_PATH(zephyr_user)
#define NUMBER_OF_GATES     2


/* *****************************************************************************
 * Debugging
 */

#if CONFIG_FKMG_SEQ_NO_OPTIMIZATIONS
#pragma GCC push_options
#pragma GCC optimize ("Og")
#warning "sequencer.c unoptimized!"
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sequencer, CONFIG_FKMG_SEQ_LOG_LEVEL);


/* *****************************************************************************
 * Structs
 */

static struct sequencer_module_data sequencer_md = {0};
/* Convenience accessor to keep name short: md - module data. */
#define md sequencer_md



/* *****************************************************************************
 * Device Structs
 */

const struct gpio_dt_spec gates[NUMBER_OF_GATES] = {
    GPIO_DT_SPEC_GET(GATE_PINS, gate_high_gpios), // Gate 0
    GPIO_DT_SPEC_GET(GATE_PINS, gate_low_gpios)   // Gate 1
};

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

// static enum Sequencer_Id next_seq_channel(enum Sequencer_Id id)
// {
//     return((id + 1) % k_Seq_Id_Cnt);
// }

static enum Sequencer_Step_Id next_step(struct Sequencer_Instance * p_inst, enum Sequencer_Step_Id id)
{
    uint8_t step = p_inst->seq.step[id]; 
    return((step + 1) % k_Seq_Step_Id_Cnt);
}

static void reset_timer(struct Sequencer_Instance * p_inst, 
                        enum Sequencer_Id id, 
                        int tix)
{

    struct counter_alarm_cfg timer_cfg = {
        .flags = 0,
        .ticks = tix,
        .callback = p_inst->tim_cb[id],
        .user_data = p_inst
    }; 
    if (counter_set_channel_alarm(p_inst->timer.t[id], 0, &timer_cfg)) {
        // LOG_ERR("Error: Failed to set Timer Y"); 
    };  
}

static uint16_t calculate_gate_timer_delay(struct Sequencer_Instance * p_inst, enum Sequencer_Id id, bool edge) 
{
    /* when we enter this function, we first need to check to see if we're on a rising or a falling edge */ 
    uint16_t delay_val; 

    if (!edge) {
        /* read the next time value of this particular channel */
        delay_val = p_inst->seq.time[next_step(&p_inst, id)]; 

        /* once we have that value, we divide it by two */
        delay_val = delay_val >> 1; 
            
        /* set our edge information */
        p_inst->seq.edge[id] = true; 
            
        /* save value in delay buffer */
        p_inst->seq.delay_buffer[id] = delay_val; 

        /* return value */
        return delay_val; 
    } 

    p_inst->seq.edge[id] = false; 
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

static void init_listener(struct Seq_Listener * p_lsnr)
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
        printk("Timers Config: FAILED.\n");
        return;
    } else {
        printk("Timers Config: PASSED\n");
    }
    

    struct counter_alarm_cfg x_cfg = {
        .flags = 0,
        .ticks = 1000,
        .callback = p_inst->tim_cb[0],
        .user_data = p_inst
    }; 

    struct counter_alarm_cfg y_cfg = {
        .flags = 0,
        .ticks = 1000,
        .callback = p_inst->tim_cb[1],
        .user_data = p_inst
    }; 
    
    if (counter_set_channel_alarm(p_inst->timer.t[0], 0, &x_cfg) != 0) {
        // LOG_ERR("Error: Failed to set Timer X"); 
    };

    if (counter_set_channel_alarm(p_inst->timer.t[1], 0, &y_cfg) != 0) {
        // LOG_ERR("Error: Failed to set Timer Y"); 
    };

    p_inst->seq.edge[0] = 0; 
    p_inst->seq.edge[1] = 0; 

    counter_start(p_inst->timer.t[0]);
    counter_start(p_inst->timer.t[1]);

    return; 

}


static void init_gate_gpios(struct Sequencer_Instance * p_inst) 
{
    for (int i = 0; i < NUMBER_OF_GATES; i++) {
        if (!device_is_ready(gates[i].port)) {
            printk("GATE %d Init : FAILED.\n", i);
        } else {
            printk("GATE %d Init : PASSED.\n", i);
            if (gpio_pin_configure_dt(&gates[i], GPIO_OUTPUT_INACTIVE)) {
                printk("GATE %d Config : FAILED.\n", i);
            } else {
                printk("GATE %d Config : PASSED.\n", i);
            }
        }
    }
}


static void config_instance_deferred(
        struct Sequencer_Instance     * p_inst,
        struct Sequencer_Instance_Cfg * p_cfg)
{
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

static void broadcast_led_driver_write_ready(
        struct Sequencer_Instance * p_inst,
        enum Sequencer_Id           id,
        uint16_t              val)
{
    struct Sequencer_Evt evt = {
            .sig = k_Seq_Evt_Sig_LED_Write_Ready,
            .data.led_write.id = id,
            .data.led_write.val = val
    };

    broadcast_event_to_listeners(p_inst, &evt);
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

static bool signal_has_listeners(
        struct Sequencer_Instance * p_inst,
        enum Sequencer_Evt_Sig      sig)
{
    return(!sys_slist_is_empty(&p_inst->list.listeners[sig]));
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
        .data.stepped.edge = false
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



static void check_current_step(struct Sequencer_Instance * p_inst, enum Pot_Id id) 
{

    uint8_t ch;
    uint8_t stp; 

    switch (id) {

        case 32:
        case 33:
            /* these pots deal with time scale, so if we detect a change here we'll have to update our timers as well */
            return;
            break;
        case 34:
            /* Some parameter, not quite sure what it does yet, might be clock divider. Only time will tell. */
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
  
    /* TODO: would we also need to update midi out in either case, since we'd have to send a note on/off message regardless? */

}


static void set_voltage_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    uint8_t channel = id + 1;

    dac_write_new_value(channel, p_inst->seq.voltage[p_inst->seq.step[id]]); 

}

static void set_gate_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    if (id < 0 || id >= NUMBER_OF_GATES) return;

    bool edge = p_inst->seq.edge[id];
    int result = gpio_pin_set_dt(&gates[id], !edge);

    if (result < 0) printk("Gate %d Error: %d\n", id, result);
}

static void set_midi_on_step(struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{

    uint8_t midi_ch_base = p_inst->seq.edge[id] ? 0x80 : 0x90; 
    uint8_t midi_status_ch = midi_ch_base | id;
    uint8_t last_note = p_inst->midi.last_note[id];
    uint16_t raw_voltage = p_inst->seq.voltage[p_inst->seq.step[id]]; 
    uint8_t ctrl_byte = 0x75; 

    struct Sequencer_Evt evt = {
        .sig = k_UART_Evt_Sig_Write_Ready,
        .data.midi_write.id = id,
        .data.midi_write.midi_status = midi_status_ch,
        .data.midi_write.raw_voltage = raw_voltage,
        .data.midi_write.last_note = last_note,
        .data.midi_write.ctrl_byte = ctrl_byte
    };

    /* Broadcast to MIDI/UART listener that we're ready to write and have data available */
    broadcast_event_to_listeners(p_inst, &evt);

}

static void set_ui_on_step (struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{
    uint16_t ui_data = 0; 

    ui_data |= (1U << p_inst->seq.step[id]); 

    /* Broadcast to UI listener that we're ready to write and have data available */

}

static void advance_sequencer_step (struct Sequencer_Instance * p_inst, enum Sequencer_Id id) 
{

    if (p_inst->seq.maxStep[id] == 0) return;

    uint8_t next_step = (p_inst->seq.step[id] + 1) % p_inst->seq.maxStep[id];

    p_inst->seq.step[id] = next_step; 

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


    p_inst->seq.voltage[0] = 1024; 

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
            
            set_voltage_on_step(p_inst, p_stepped->id);
            set_gate_on_step(p_inst, p_stepped->id);
            set_ui_on_step (p_inst, p_stepped->id);
            set_midi_on_step (p_inst, p_stepped->id);

            uint16_t new_tix = calculate_gate_timer_delay(p_inst, p_evt->data.stepped.id, p_inst->seq.edge[p_evt->data.stepped.id]);
            if (new_tix < 1) new_tix = 1;

            reset_timer(p_inst, p_stepped->id, new_tix);

            advance_sequencer_step(&p_inst, p_stepped->id);

            break; 
        case k_Seq_SM_Evt_Sig_Pot_Value_Changed:
            struct Sequencer_SM_Evt_Sig_Pot_Value_Changed * p_pot_changed = &p_evt->data.pot_changed; 
            
            post_updated_pot_value(&p_inst, p_pot_changed->pot_id, p_pot_changed->pot_id);
            check_current_step(&p_inst, p_pot_changed->pot_id);

            break;
        case k_Seq_SM_Evt_Sig_Btn_Status_Changed:
            struct Sequencer_SM_Evt_Sig_Btn_Status_Changed * p_btn_changed = &p_evt->data.btn_changed; 
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
