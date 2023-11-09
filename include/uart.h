/*******************************************************************************
 * @brief UART interface.
 *
 * This module is the public api to interface with UART.
 *
 * @example
 */

#ifndef FKMG_UART_H
#define FKMG_UART_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "uart/instance.h"
#include "uart/instance_cfg.h"
#include "uart/listener.h"
#include "uart/listener_cfg.h"

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
void UART_Init_Instance(struct UART_Instance_Cfg * p_cfg);

/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_UART_ALLOW_SHUTDOWN
void UART_Deinit_Instance(struct UART_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void UART_Add_Listener(struct UART_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_UART_ALLOW_LISTENER_REMOVAL
void UART_Remove_Listener(struct UART_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_UART_H */
