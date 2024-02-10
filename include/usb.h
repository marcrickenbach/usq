/*******************************************************************************
 * @brief USB Thread.
 *
 * This module is the public api to interface with USB. 
 *
 * @example
 */

#ifndef FKMG_USB_H
#define FKMG_USB_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "usb/instance.h"
#include "usb/instance_cfg.h"
#include "usb/listener.h"
#include "usb/listener_cfg.h"


/* *****************************************************************************
 * Defines
 */


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
void USB_Init_Instance(struct USB_Instance_Cfg * p_cfg);


/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_USB_ALLOW_SHUTDOWN
void USB_Deinit_Instance(struct USB_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void USB_Add_Listener(struct USB_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_USB_ALLOW_LISTENER_REMOVAL
void USB_Remove_Listener(struct USB_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_USB_H */
