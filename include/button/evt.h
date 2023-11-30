/** ****************************************************************************
 * @brief Events generated by Button module.
 *
 * The public events Button module generates.
 */

#ifndef FKMG_BUTTON_EVT_H
#define FKMG_BUTTON_EVT_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "id.h"

/* *****************************************************************************
 * Events
 */

/* Signals that can be generated as part of an event. */
enum Button_Evt_Sig{
    k_Button_Evt_Sig_Beg,                        // Inclusive
    k_Button_Evt_Sig_Min = k_Button_Evt_Sig_Beg,    // Inclusive
    k_Button_Evt_Sig_1st = k_Button_Evt_Sig_Beg,    // Inclusive

    k_Button_Evt_Sig_Instance_Initialized = k_Button_Evt_Sig_Beg,
    #if CONFIG_FKMG_Button_SHUTDOWN_ENABLED
    k_Button_Evt_Sig_Instance_Deinitialized,
    #endif
    k_Button_Evt_Sig_Pressed,                       //
    k_Button_Evt_Sig_Released,                      //
    k_Button_Evt_Sig_Debounced,                     // 
         

    k_Button_Evt_Sig_End,                        // Exclusive
    k_Button_Evt_Sig_Max = k_Button_Evt_Sig_End - 1,// Inclusive
    k_Button_Evt_Sig_Lst = k_Button_Evt_Sig_End - 1,// Inclusive
    k_Button_Evt_Sig_Cnt = k_Button_Evt_Sig_End
                      - k_Button_Evt_Sig_Beg,	  // Inclusive
	k_Button_Evt_Sig_Ivd = k_Button_Evt_Sig_End
};

/* Forward references to prevent include interdependent items getting declared
 * out-of-order. */
struct Button_Instance;

/* Data signal k_DAC_Evt_Sig_Instance_Initialized can generate. */
struct Button_Evt_Data_Instance_Initialized{
    struct Button_Instance * p_inst;
};

/* Data signal k_Button_Evt_Sig_Pressed can generate. */
struct Button_Evt_Data_Pressed{
    uint8_t portA_state;
    uint8_t portB_state;
    int64_t timestamp;
};

/* Events (i.e. signal + signal's data if any) that can be generated. */
struct Button_Evt{
	enum Button_Evt_Sig sig;
	union Button_Evt_Data{
        struct Button_Evt_Data_Instance_Initialized     initd;
        struct Button_Evt_Data_Pressed                  pressed; 
	}data;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_BUTTON_EVT_H */
