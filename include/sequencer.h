/*******************************************************************************
 * @brief Sequencer interface.
 *
 * This module is the public api to interface with the main Sequencer. 
 * The Sequencer oversees pot changes, sends dac write commands and handles timing of steps. 
 *
 * @example
 */

#ifndef FKMG_SEQUENCER_H
#define FKMG_SEQUENCER_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "sequencer/instance.h"
#include "sequencer/instance_cfg.h"
#include "sequencer/listener.h"
#include "sequencer/listener_cfg.h"

/* *****************************************************************************
 * Public
 */

/* NOTE: all callbacks must be handled in a timely manner since the callbacks
 * occur on the thread of this engine. */

/**
 * Initialize an instance. An instance contains everything required to declare
 * and define all the data/memory for threads, queues, etc., that runs the the
 * engine. Even though it might be possible to have multiple instances,
 * typically only one is required.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void Sequencer_Init_Instance(struct Sequencer_Instance_Cfg * p_cfg);

// static void on_pot_change_ext(struct Sequencer_Evt *p_evt); 
/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
void Sequencer_Deinit_Instance(struct Sequencer_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void Sequencer_Add_Listener(struct Sequencer_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_SEQ_ALLOW_LISTENER_REMOVAL
void Sequencer_Remove_Listener(struct Sequencer_Listener * p_lsnr);
#endif


/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQUENCER_H */
