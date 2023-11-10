/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#include <assert.h>

#include "main.h"

#include "dac.h"
#include "dac/private/sm_evt.h"

#include "led_driver.h"
#include "led_driver/private/sm_evt.h"

#include "pot.h"
#include "pot/id.h"
#include "pot/private/sm_evt.h"

#include "sequencer.h"
#include "sequencer/private/sm_evt.h"

#include "uart.h"
#include "uart/private/sm_evt.h"


/* *****************************************************************************
 * Defines
 */

/* Error if took longer than this to initialize. */
#define WAIT_MAX_MSECS_FOR_INITIALIZATION 50


/* *****************************************************************************
 * Threads
 */

/* Declare threads, queues, and other data structures for pot instance. */
static struct k_thread pot_thread;
#define POT_THREAD_STACK_SZ_BYTES   512
K_THREAD_STACK_DEFINE(pot_thread_stack, POT_THREAD_STACK_SZ_BYTES);
#define MAX_QUEUED_POT_SM_EVTS  10
#define POT_SM_QUEUE_ALIGNMENT  4
K_MSGQ_DEFINE(pot_sm_evt_q, sizeof(struct Pot_SM_Evt),
        MAX_QUEUED_POT_SM_EVTS, POT_SM_QUEUE_ALIGNMENT);
static struct Pot_Instance pot_inst;

/* Declare threads, queues, and other data structures for DAC instance. */
static struct k_thread dac_thread;
#define DAC_THREAD_STACK_SZ_BYTES   512
K_THREAD_STACK_DEFINE(dac_thread_stack, DAC_THREAD_STACK_SZ_BYTES);
#define MAX_QUEUED_DAC_SM_EVTS  10
#define DAC_SM_QUEUE_ALIGNMENT  4
K_MSGQ_DEFINE(dac_sm_evt_q, sizeof(struct DAC_SM_Evt),
        MAX_QUEUED_DAC_SM_EVTS, DAC_SM_QUEUE_ALIGNMENT);
static struct DAC_Instance dac_inst;

/* Declare threads, queues, and other data structures for Sequencer instance. */
static struct k_thread sequencer_thread;
#define SEQUENCER_THREAD_STACK_SZ_BYTES   512
K_THREAD_STACK_DEFINE(sequencer_thread_stack, SEQUENCER_THREAD_STACK_SZ_BYTES);
#define MAX_QUEUED_SEQUENCER_SM_EVTS  10
#define SEQUENCER_SM_QUEUE_ALIGNMENT  4
K_MSGQ_DEFINE(sequencer_sm_evt_q, sizeof(struct Sequencer_SM_Evt),
        MAX_QUEUED_SEQUENCER_SM_EVTS, SEQUENCER_SM_QUEUE_ALIGNMENT);
static struct Sequencer_Instance sequencer_inst;

/* Declare threads, queues, and other data structures for LED Driver instance. */
static struct k_thread led_driver_thread;
#define LED_DRIVER_THREAD_STACK_SZ_BYTES   512
K_THREAD_STACK_DEFINE(led_driver_thread_stack, LED_DRIVER_THREAD_STACK_SZ_BYTES);
#define MAX_QUEUED_LED_DRIVER_SM_EVTS  10
#define LED_DRIVER_SM_QUEUE_ALIGNMENT  4
K_MSGQ_DEFINE(led_driver_sm_evt_q, sizeof(struct LED_Driver_SM_Evt),
        MAX_QUEUED_LED_DRIVER_SM_EVTS, LED_DRIVER_SM_QUEUE_ALIGNMENT);
static struct LED_Driver_Instance led_driver_inst;

/* Declare threads, queues, and other data structures for UART / MIDI instance. */
static struct k_thread uart_thread;
#define UART_THREAD_STACK_SZ_BYTES   512
K_THREAD_STACK_DEFINE(uart_thread_stack, UART_THREAD_STACK_SZ_BYTES);
#define MAX_QUEUED_UART_SM_EVTS  10
#define UART_SM_QUEUE_ALIGNMENT  4
K_MSGQ_DEFINE(uart_sm_evt_q, sizeof(struct UART_SM_Evt),
        MAX_QUEUED_UART_SM_EVTS, UART_SM_QUEUE_ALIGNMENT);
static struct UART_Instance uart_inst;

/* *****************************************************************************
 * Listeners
 */

#if 0
static struct Pot_Listener midi_rcvd_listener;
#endif

/* *****************************************************************************
 * Kernel Events
 *
 * Zephyr low-overhead kernel way for threads to wait on and post asynchronous
 * events.
 */

/* For handling asynchronous callbacks. */
K_EVENT_DEFINE(events);
#define EVT_FLAG_INSTANCE_INITIALIZED   (1 << 0)



/* *****************************************************************************
 * Private Functions.
 */

/* ********************
 * ON INSTANCE INITS
 * ********************/

static void on_pot_instance_initialized(struct Pot_Evt *p_evt)
{
    assert(p_evt->sig == k_Pot_Evt_Sig_Instance_Initialized);
    assert(p_evt->data.initd.p_inst == &pot_inst);
	k_event_post(&events, EVT_FLAG_INSTANCE_INITIALIZED);
}

static void on_dac_instance_initialized(struct DAC_Evt *p_evt)
{
    assert(p_evt->sig == k_DAC_Evt_Sig_Instance_Initialized);
    assert(p_evt->data.initd.p_inst == &dac_inst);
	k_event_post(&events, EVT_FLAG_INSTANCE_INITIALIZED);
}

static void on_sequencer_instance_initialized(struct Sequencer_Evt *p_evt)
{
    assert(p_evt->sig == k_Seq_Evt_Sig_Instance_Initialized);
    assert(p_evt->data.initd.p_inst == &sequencer_inst);
	k_event_post(&events, EVT_FLAG_INSTANCE_INITIALIZED);
}

static void on_led_driver_instance_initialized(struct LED_Driver_Evt *p_evt)
{
    assert(p_evt->sig == k_LED_Driver_Evt_Sig_Instance_Initialized);
    assert(p_evt->data.initd.p_inst == &led_driver_inst);
	k_event_post(&events, EVT_FLAG_INSTANCE_INITIALIZED);
}

static void on_uart_instance_initialized(struct UART_Evt *p_evt)
{
    assert(p_evt->sig == k_UART_Evt_Sig_Instance_Initialized);
    assert(p_evt->data.initd.p_inst == &uart_inst);
	k_event_post(&events, EVT_FLAG_INSTANCE_INITIALIZED);
}

static void wait_on_instance_initialized(void)
{
	uint32_t events_rcvd = k_event_wait(&events, EVT_FLAG_INSTANCE_INITIALIZED,
		true, K_MSEC(WAIT_MAX_MSECS_FOR_INITIALIZATION));
	assert(events_rcvd == EVT_FLAG_INSTANCE_INITIALIZED);
}


  

/* ********************
 * ON POT CHANGE
 * ********************/

static void on_pot_changed(struct Pot_Evt *p_evt)
{
    assert(p_evt->sig == k_Pot_Evt_Sig_Changed);
    struct Sequencer_SM_Evt_Sig_Pot_Value_Changed * p_changed = &p_evt->data.changed;

    struct Sequencer_SM_Evt evt = {
        .sig = k_Seq_SM_Evt_Sig_Pot_Value_Changed,
        .data.pot_changed = *p_changed
    }; 

    /* Pass along event to the sequencer state machine. 
       There we'll update our sequencer values and test to see if we're in the middle of a step
       which the potentiometer controls. If so, we can take care of any necessary adjustments (DAC or changing the timer)
    */
    k_msgq_put(&sequencer_sm_evt_q, &evt, K_NO_WAIT);

}

static void on_led_write_ready(struct LED_Driver_Evt *p_evt) 
{
    assert(p_evt->sig == k_Seq_Evt_Sig_LED_Write_Ready);

    struct LED_Driver_SM_Evt evt = {
            .sig = k_LED_Driver_SM_Evt_LED_Driver_Write,
            // .data.write = &p_evt->data.write
    };

    evt.data.write.id = p_evt->data.write.id;
    evt.data.write.val = p_evt->data.write.val;

    /*  pass along event for SM to handle */
    k_msgq_put(&led_driver_sm_evt_q, &evt, K_NO_WAIT); 
    
}

static void on_midi_write_ready(struct UART_Evt *p_evt) 
{
    assert(p_evt->sig == k_UART_Evt_Sig_Write_Ready);

    struct UART_SM_Evt_Sig_Write_MIDI * evt_cfg = &p_evt->data.midi_write; 

    enum UART_Id id = p_evt->data.midi_write.id; 

    struct UART_SM_Evt evt = {
            .sig = k_UART_SM_Evt_Sig_Write_MIDI,
            .data.midi_write.id = id,
            .data.midi_write.midi_status = p_evt->data.midi_write.midi_status,
            .data.midi_write.raw_voltage = p_evt->data.midi_write.raw_voltage,
            .data.midi_write.last_note = p_evt->data.midi_write.last_note,
            .data.midi_write.ctrl_byte = p_evt->data.midi_write.ctrl_byte
    };

    /*  pass along event for SM to handle */
    k_msgq_put(&uart_sm_evt_q, &evt, K_NO_WAIT); 
    
}



/* *****************************************************************************
 * Public.
 */

static void pre_populate_adc_values(void) 
{
    for (int i = 0; i < ARRAY_SIZE(pot_inst.adc_current_reading); ++i)
    {
        pot_inst.adc_current_reading[i] = 1024;
    }
}


      int main (void) {
    // /* Instance: Pot */
    struct Pot_Instance_Cfg pot_inst_cfg = {
        .p_inst = &pot_inst,
        .task.sm.p_thread = &pot_thread,
        .task.sm.p_stack = pot_thread_stack,
        .task.sm.stack_sz = K_THREAD_STACK_SIZEOF(pot_thread_stack),
        .task.sm.prio = K_LOWEST_APPLICATION_THREAD_PRIO,
        .msgq.p_sm_evts = &pot_sm_evt_q,
        .cb = on_pot_instance_initialized,
    };
    Pot_Init_Instance(&pot_inst_cfg);
    wait_on_instance_initialized();


    static struct Pot_Listener pot_changed_lsnr;
    struct Pot_Listener_Cfg pot_lsnr_cfg = {
        .p_inst = &pot_inst,
        .p_lsnr = &pot_changed_lsnr, 
        .sig     = k_Pot_Evt_Sig_Changed,
        .cb      = on_pot_changed
    };
    Pot_Add_Listener(&pot_lsnr_cfg);


    /* Instance: DAC */
    struct DAC_Instance_Cfg dac_inst_cfg = {
        .p_inst = &dac_inst,
        .task.sm.p_thread = &dac_thread,
        .task.sm.p_stack = dac_thread_stack,
        .task.sm.stack_sz = K_THREAD_STACK_SIZEOF(dac_thread_stack),
        .task.sm.prio = K_LOWEST_APPLICATION_THREAD_PRIO,
        .msgq.p_sm_evts = &dac_sm_evt_q,
        .cb = on_dac_instance_initialized,
    };
    DAC_Init_Instance(&dac_inst_cfg);
    wait_on_instance_initialized();


     /* Instance: LED Driver */
    struct LED_Driver_Instance_Cfg led_driver_inst_cfg = {
        .p_inst = &led_driver_inst,
        .task.sm.p_thread = &led_driver_thread,
        .task.sm.p_stack = led_driver_thread_stack,
        .task.sm.stack_sz = K_THREAD_STACK_SIZEOF(led_driver_thread_stack),
        .task.sm.prio = K_LOWEST_APPLICATION_THREAD_PRIO,
        .msgq.p_sm_evts = &led_driver_sm_evt_q,
        .cb = on_led_driver_instance_initialized,
    };
    LED_Driver_Init_Instance(&led_driver_inst_cfg);
    wait_on_instance_initialized();

     /* Instance: UART Module */
    struct UART_Instance_Cfg uart_inst_cfg = {
        .p_inst = &uart_inst,
        .task.sm.p_thread = &uart_thread,
        .task.sm.p_stack = uart_thread_stack,
        .task.sm.stack_sz = K_THREAD_STACK_SIZEOF(uart_thread_stack),
        .task.sm.prio = K_LOWEST_APPLICATION_THREAD_PRIO,
        .msgq.p_sm_evts = &uart_sm_evt_q,
        .cb = on_uart_instance_initialized,
    };
    UART_Init_Instance(&uart_inst_cfg);
    wait_on_instance_initialized();

    /* Instance: Sequencer */
    struct Sequencer_Instance_Cfg sequencer_inst_cfg = {
        .p_inst = &sequencer_inst,
        .task.sm.p_thread = &sequencer_thread,
        .task.sm.p_stack = sequencer_thread_stack,
        .task.sm.stack_sz = K_THREAD_STACK_SIZEOF(sequencer_thread_stack),
        .task.sm.prio = K_LOWEST_APPLICATION_THREAD_PRIO,
        .msgq.p_sm_evts = &sequencer_sm_evt_q,
        .cb = on_sequencer_instance_initialized,
    };
    Sequencer_Init_Instance(&sequencer_inst_cfg);
    wait_on_instance_initialized();
    
    static struct Sequencer_Listener led_write_lsnr;
    struct Sequencer_Listener_Cfg led_write_lsnr_cfg = {
        .p_inst = &sequencer_inst,
        .p_lsnr = &led_write_lsnr, 
        .sig     = k_Seq_Evt_Sig_LED_Write_Ready,
        .cb      = on_led_write_ready
    };
    Sequencer_Add_Listener(&led_write_lsnr_cfg);

    static struct Sequencer_Listener midi_write_lsnr;
    struct Sequencer_Listener_Cfg midi_write_lsnr_cfg = {
        .p_inst = &sequencer_inst,
        .p_lsnr = &midi_write_lsnr, 
        .sig     = k_UART_Evt_Sig_Write_Ready,
        .cb      = on_midi_write_ready
    };
    Sequencer_Add_Listener(&midi_write_lsnr_cfg);

    /* FIXME: remove when we have the pot module up and running correctly */
    pre_populate_adc_values(); 

    return 0;

};
