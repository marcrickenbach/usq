/** ****************************************************************************
 * @brief Internal state machine events generated by LED Driver.
 */

#ifndef FKMG_LED_DRIVER_SM_EVT_H
#define FKMG_LED_DRIVER_SM_EVT_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "../id.h"
#include "../instance_cfg.h"


/* *****************************************************************************
 * Events
 */

/* Signals that can be generated as part of an event. */
enum LED_Driver_SM_Evt_Sig{
    k_LED_Driver_SM_Evt_Sig_Beg,                           // Inclusive
    k_LED_Driver_SM_Evt_Sig_Min = k_LED_Driver_SM_Evt_Sig_Beg,    // Inclusive
    k_LED_Driver_SM_Evt_Sig_1st = k_LED_Driver_SM_Evt_Sig_Beg,    // Inclusive

	k_LED_Driver_SM_Evt_Sig_Init_Instance = k_LED_Driver_SM_Evt_Sig_Beg,
    #if CONFIG_FKMG_SEQ_ALLOW_SHUTDOWN
	k_LED_Driver_SM_Evt_Sig_Deinit_Instance,
    #endif
    // k_Seq_Evt_Sig_Pot_Changed,
    k_LED_Driver_SM_Evt_LED_Driver_Write,
    k_LED_Driver_SM_Evt_Change_Default_Levels,
    k_LED_Driver_SM_Evt_LED_Driver_Reset_LED,
    k_LED_Driver_SM_Evt_Sig_End,                           // Exclusive
    k_LED_Driver_SM_Evt_Sig_Max = k_LED_Driver_SM_Evt_Sig_End - 1,// Inclusive
    k_LED_Driver_SM_Evt_Sig_Lst = k_LED_Driver_SM_Evt_Sig_End - 1,// Inclusive
    k_LED_Driver_SM_Evt_Sig_Cnt = k_LED_Driver_SM_Evt_Sig_End
                         - k_LED_Driver_SM_Evt_Sig_Beg,	// Inclusive
	k_LED_Driver_SM_Evt_Sig_Ivd = k_LED_Driver_SM_Evt_Sig_End
};

/* Data signal k_LED_Driver_SM_Evt_Sig_Init_Instance can generate. */
struct LED_Driver_SM_Evt_Sig_Init_Instance{
    struct LED_Driver_Instance_Cfg cfg;
};

/* Data signal k_LED_Driver_SM_Evt_LED_Driver_Reset_LED can generate. */
struct LED_Driver_SM_Evt_Sig_LED_Driver_Reset_LED{
    bool channel;
    uint16_t step;
    uint8_t offset;
    uint16_t val; 
};

struct LED_Driver_SM_Evt_Sig_Change_Default_Levels{
    uint8_t btn_id;
    bool armed;
    uint8_t offset; 
    uint8_t step; 
    bool edge; 
};

/* Events (i.e. signal + signal's data if any) Seq State Machine generates. */
struct LED_Driver_SM_Evt{
	enum LED_Driver_SM_Evt_Sig sig;
	union LED_Driver_SM_Evt_Data{
        struct LED_Driver_SM_Evt_Sig_Init_Instance          init_inst;
        struct LED_Driver_SM_Evt_Sig_LED_Driver_Reset_LED   reset;
        struct LED_Driver_SM_Evt_Sig_Change_Default_Levels  def_lvl;
	}data;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_LED_DRIVER_SM_EVT_H */
