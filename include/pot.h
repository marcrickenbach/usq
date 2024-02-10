/*******************************************************************************
 * @brief Potentiometer interface.
 *
 * This module is the public api to interface with potentiometers. The pots are
 * ...
 *
 * @example
 */

#ifndef FKMG_POT_H
#define FKMG_POT_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "pot/instance.h"
#include "pot/instance_cfg.h"
#include "pot/listener.h"
#include "pot/listener_cfg.h"

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
void Pot_Init_Instance(struct Pot_Instance_Cfg * p_cfg);

/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_POT_ALLOW_SHUTDOWN
void Pot_Deinit_Instance(struct Pot_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void Pot_Add_Listener(struct Pot_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_POT_ALLOW_LISTENER_REMOVAL
void Pot_Remove_Listener(struct Pot_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 * Get_Pot(pot id): get the last read value of the specified pot. Do not read
 *   the adc! This is useful for getting the initial value of a pot at startup
 *   etc., without having to wait for a conversion.
 *
 * Force_Pot_Change_Evt(pot id): force event with signal k_Pot_Sig_Changed to
 *   occur. Value isn't changed; it is a way to force an event if asynchronous
 *   listeners need current value.
 */



#ifdef __cplusplus
}
#endif

#endif /* FKMG_POT_H */
