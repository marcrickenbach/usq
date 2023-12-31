/** ****************************************************************************
 * @brief Internal state machine events generated by BUTTON.
 */

#ifndef FKMG_BUTTON_SM_EVT_H
#define FKMG_BUTTON_SM_EVT_H

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
enum Button_SM_Evt_Sig{
    k_Button_SM_Evt_Sig_Beg,                           // Inclusive
    k_Button_SM_Evt_Sig_Min = k_Button_SM_Evt_Sig_Beg,    // Inclusive
    k_Button_SM_Evt_Sig_1st = k_Button_SM_Evt_Sig_Beg,    // Inclusive

	k_Button_SM_Evt_Sig_Init_Instance = k_Button_SM_Evt_Sig_Beg,
    #if CONFIG_FKMG_BUTTON_ALLOW_SHUTDOWN
	k_Button_SM_Evt_Sig_Deinit_Instance,
    #endif
	k_Button_SM_Evt_Sig_Pressed,
    k_Button_SM_Evt_Sig_Released,
    k_Button_SM_Evt_Sig_Do_Debounce,
    k_Button_SM_Evt_Sig_Debounced,

    k_Button_SM_Evt_Sig_End,                           // Exclusive
    k_Button_SM_Evt_Sig_Max = k_Button_SM_Evt_Sig_End - 1,// Inclusive
    k_Button_SM_Evt_Sig_Lst = k_Button_SM_Evt_Sig_End - 1,// Inclusive
    k_Button_SM_Evt_Sig_Cnt = k_Button_SM_Evt_Sig_End
                         - k_Button_SM_Evt_Sig_Beg,	// Inclusive
	k_Button_SM_Evt_Sig_Ivd = k_Button_SM_Evt_Sig_End
};

/* Data signal k_BUTTON_SM_Evt_Sig_Init_Instance can generate. */
struct Button_SM_Evt_Sig_Init_Instance{
    struct Button_Instance_Cfg cfg;
};

/* Data signal k_Button_SM_Evt_Sig_Pressed can generate. */
struct Button_SM_Evt_Sig_Pressed{

};


/* Data signal k_Button_SM_Evt_Sig_Pressed can generate. */
struct Button_SM_Evt_Sig_Debounced{
    uint8_t portA_state;
    uint8_t portB_state;
};




/* Events (i.e. signal + signal's data if any) Button State Machine generates. */
struct Button_SM_Evt{
	enum Button_SM_Evt_Sig sig;
	union Button_SM_Evt_Data{
        struct Button_SM_Evt_Sig_Init_Instance  init_inst;
        struct Button_SM_Evt_Sig_Pressed        pressed;
        struct Button_SM_Evt_Sig_Debounced      debounced;
	}data;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_BUTTON_SM_EVT_H */
