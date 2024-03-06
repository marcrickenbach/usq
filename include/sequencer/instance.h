/** ****************************************************************************
 * @brief Seq instance interface.
 *
 * An instance is the root instantiation of the implementation. An instance
 * contains everything required to declare and define all the threads, queues,
 * etc. Even though it might be possible to have multiple instances, typically
 * only one is required or even possible.
 */

#ifndef FKMG_SEQ_INSTANCE_H
#define FKMG_SEQ_INSTANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>

#include "id.h"
#include "step_id.h"
#include "err.h"
#include "evt.h"
#include "private/sm_evt.h"

/* *****************************************************************************
 * Enums
 */

/* *****************************************************************************
 * Instance
 */

struct Sequencer_Instance{
    #if CONFIG_FKMG_SEQ_RUNTIME_ERROR_CHECKING
    /* Error status. */
	enum Sequencer_Err_Id err;
    #endif

    /* Threads used. */
    struct{
        struct{
            struct k_thread * p_thread;
        }sm;
    }task;

    /* Queues used. */
    struct{
        /* State machine event message queue. */
        struct k_msgq * p_sm_evts;
    }msgq;

    /* State machine. */
	struct smf_ctx sm;

    /* Current sm event. */
    struct Sequencer_SM_Evt sm_evt;

    /* Singly linked lists to keep track of things. */
    struct{
        sys_slist_t listeners[k_Seq_Evt_Sig_Cnt];
    }list;

    /* For adding this instance to singly linked lists. */
    struct{
        sys_snode_t instance;
    }node;

    /* Timers used. */
    struct{
        const struct device *t[2]; 
        bool running[2];
        struct k_timer mode_btn;
        struct k_timer mode_led;
    }timer;

    struct {
        uint8_t last_note[2];
    }midi;

        /* offset in following struct tells us where sequencer channel 2 begins. 
           don't use maxStep since once a sequencer's total possible steps is set,
           user still has possibility to deactivate, say, sequencer 1 steps and probably
           doesn't want sequencer two shifted up. 
        */
    struct{
        uint8_t step[2];
        uint8_t minStep[2];
        uint8_t maxStep[2];
        bool active[16]; 
        uint16_t voltage[16];
        uint16_t time[16]; 
        float param[2]; 
        uint16_t delay_buffer[2]; 
        bool edge[2]; 
        bool random[2];
        bool direction[2]; 
        bool running[2];
        uint16_t global;
        uint16_t slew; 
        uint8_t offset; 
        uint8_t mode; 
        uint8_t state; 
        bool shift; 
    }seq;

    counter_alarm_callback_t tim_cb[2];

    /* Current Seq. */
    enum Sequencer_Id id;

    struct gpio_callback transport_gpio_1_cb;
    struct gpio_callback transport_gpio_2_cb;
    struct gpio_callback transport_gpio_1_rst_cb;
    struct gpio_callback transport_gpio_2_rst_cb;

};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_Seq_INSTANCE_H */
