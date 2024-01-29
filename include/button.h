/*******************************************************************************
 * @brief Button Thread.
 *
 * This module is the public api to interface with buttons. 
 *
 * @example
 */

#ifndef FKMG_BUTTON_H
#define FKMG_BUTTON_H

/* In case C++ needs to use anything here */
#ifdef __cplusplus
extern “C” {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "button/instance.h"
#include "button/instance_cfg.h"
#include "button/listener.h"
#include "button/listener_cfg.h"


/* *****************************************************************************
 * Defines
 */

#define MCP23017_ADDR           0x20
#define MCP23017_CTRL_WRITE     0x40
#define MCP23017_CTRL_READ      0x41

#define CONFIG_BUTTON_8_BIT_MODE    1

#if CONFIG_BUTTON_8_BIT_MODE

    #define MCP23017_IODIRA     0x00        // I/O direction register for port A
    #define MCP23017_IODIRB     0x01        // I/O direction register for port B
    #define MCP23017_IPOLA      0x02        // Input polarity port register A
    #define MCP23017_IPOLB      0x03        // Input polarity port register B
    #define MCP23017_GPINTENA   0x04        // Interrupt-on-change control register for port A
    #define MCP23017_GPINTENB   0x05        // Interrupt-on-change control register for port B
    #define MCP23017_DEFVALA    0x06        // Default value for port A
    #define MCP23017_DEFVALB    0x07        // Default value for port B
    #define MCP23017_INTCONA    0x08        // Interrupt control register for port A
    #define MCP23017_INTCONB    0x09        // Interrupt control register for port B
    #define MCP23017_IOCONA     0x0A        // IOCON register
    #define MCP23017_IOCONB     0x0B        // IOCON register
    #define MCP23017_GPPUA      0x0C        // Pull-up resistor register for port A
    #define MCP23017_GPPUB      0x0D        // Pull-up resistor register for port B
    #define MCP23017_INTFA      0x0E        // Interrupt condition register for port A
    #define MCP23017_INTFB      0x0F        // Interrupt condition register for port B
    #define MCP23017_INTCAPA    0x10        // Interrupt capture register for port A
    #define MCP23017_INTCAPB    0x11        // Interrupt capture register for port B
    #define MCP23017_GPIOA      0x12        // GPIO register for port A
    #define MCP23017_GPIOB      0x13        // GPIO register for port B
    #define MCP23017_OLATA      0x14        // Output latch register for port A
    #define MCP23017_OLATB      0x15        // Output latch register for port B

#else // in 16-bit Mode

    #define MCP23017_IODIRA     0x00        // I/O direction register for port A
    #define MCP23017_IOPOLA     0x01        // Input polarity port register A
    #define MCP23017_GPINTENA   0x02        // Interrupt-on-change control register for port A
    #define MCP23017_DEFVALA    0x03        // Default value for port A
    #define MCP23017_INTCONA    0x04        // Interrupt control register for port A
    #define MCP23017_IOCON      0x05        // IOCON register
    #define MCP23017_GPPUA      0x06        // Pull-up resistor register for port A
    #define MCP23017_INTFA      0x07        // Interrupt condition register for port A
    #define MCP23017_INTCAPA    0x08        // Interrupt capture register for port A
    #define MCP23017_GPIOA      0x09        // GPIO register for port A
    #define MCP23017_OLATA      0x0A        // Output latch register for port A
    #define MCP23017_IODIRB     0x10        // I/O direction register for port B
    #define MCP23017_IOPOLB     0x11        // Input polarity port register B
    #define MCP23017_GPINTENB   0x12        // Interrupt-on-change control register for port B
    #define MCP23017_DEFVALB    0x13        // Default value for port B
    #define MCP23017_INTCONB    0x14        // Interrupt control register for port B
    #define MCP23017_IOCON      0x15        // IOCON register
    #define MCP23017_GPPUB      0x16        // Pull-up resistor register for port B
    #define MCP23017_INTFB      0x17        // Interrupt condition register for port B
    #define MCP23017_INTCAPB    0x18        // Interrupt capture register for port B
    #define MCP23017_GPIOB      0x19        // GPIO register for port B
    #define MCP23017_OLATB      0x1A        // Output latch register for port B

#endif

#define CONFIG_ALL_OUTPUTS      0x00
#define CONFIG_ALL_INPUTS       0xFF

#define CONFIG_ALL_POL_NORM     0x00
#define CONFIG_ALL_POL_INV      0xFF

#define CONFIG_ALL_PULLDOWN     0x00
#define CONFIG_ALL_PULLUP       0xFF

#define CONFIG_INT_ON_CHG_DIS   0x00
#define CONFIG_INT_ON_CHG_EN    0xFF

#define CONFIG_DEFVAL_ALL_0     0x00
#define CONFIG_DEFVAL_ALL_1     0xFF

#define CONFIG_INTCON_ALL_PREV  0x00
#define CONFIG_INTCON_ALL_DEFV  0xFF

#define CONFIG_IOCON_INTPOL     (1U<<1)
#define CONFIG_IOCON_ODR        (1U<<2)
#define CONFIG_IOCON_HAEN       (1U<<3)
#define CONFIG_IOCON_DISSLW     (1U<<4)
#define CONFIG_IOCON_SEQOP      (1U<<5)
#define CONFIG_IOCON_MIRROR     (1U<<6)
#define CONFIG_IOCON_BANK       (1U<<7)



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
void Button_Init_Instance(struct Button_Instance_Cfg * p_cfg);


/**
 * Deinitialize an instance.
 * @param[in] p_dcfg Pointer to the filled-in deconfiguration struct. See the
 * struct for details.
 */
#if CONFIG_FKMG_BUTTON_ALLOW_SHUTDOWN
void Button_Deinit_Instance(struct Button_Instance_Dcfg * p_dcfg);
#endif

/**
 * Add an event signal listener to an interface.
 * @param[in] p_cfg Pointer to the filled-in configuration struct. See the
 * struct for details.
 */
void Button_Add_Listener(struct Button_Listener_Cfg * p_cfg);

#if CONFIG_FKMG_BUTTON_ALLOW_LISTENER_REMOVAL
void Button_Remove_Listener(struct Button_Listener * p_lsnr);
#endif

/* Public API functions to consider:
 *
 */
#ifdef __cplusplus
}
#endif

#endif /* FKMG_BUTTON_H */
