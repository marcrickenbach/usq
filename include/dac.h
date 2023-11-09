/*******************************************************************************
 * @brief DAC Thread.
 *
 * This module is the public api to interface with the dac. 
 *
 * @example
 */

#ifndef FKMG_DAC_H
#define FKMG_DAC_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "dac/instance.h"
#include "dac/instance_cfg.h"
#include "dac/listener.h"
#include "dac/listener_cfg.h"

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
void DAC_Init_Instance(struct DAC_Instance_Cfg * p_cfg);

void dac_write_new_value(uint8_t ch, uint32_t val); 

/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_DAC_ALLOW_SHUTDOWN
void DAC_Deinit_Instance(struct DAC_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void DAC_Add_Listener(struct DAC_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_DAC_ALLOW_LISTENER_REMOVAL
void DAC_Remove_Listener(struct DAC_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_H */
