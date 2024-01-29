/*******************************************************************************
 * @brief Potentiometer interface.
 *
 * This module is the public api to interface with potentiometers. The pots are
 * ...
 *
 * @example
 */

#ifndef FKMG_LED_DRIVER_H
#define FKMG_LED_DRIVER_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "led_driver/instance.h"
#include "led_driver/instance_cfg.h"
#include "led_driver/listener.h"
#include "led_driver/listener_cfg.h"

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
void LED_Driver_Init_Instance(struct LED_Driver_Instance_Cfg * p_cfg);

void write_led_data_to_spi(bool channel, uint16_t step, uint16_t offset, uint16_t value);


/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_LED_DRIVER_ALLOW_SHUTDOWN
void LED_Driver_Deinit_Instance(struct LED_Driver_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void LED_Driver_Add_Listener(struct LED_Driver_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_LED_DRIVER_ALLOW_LISTENER_REMOVAL
void LED_Driver_Remove_Listener(struct LED_Driver_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_LED_DRIVER_H */
