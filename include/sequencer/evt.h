/** ****************************************************************************
 * @brief Events generated by Sequencer module.
 *
 * The public events Sequencer module generates.
 */

#ifndef FKMG_SEQ_EVT_H
#define FKMG_SEQ_EVT_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "id.h"
#include "uart/evt.h"


/* *****************************************************************************
 * Events
 */

/* Signals that can be generated as part of an event. */
enum Sequencer_Evt_Sig{
    k_Seq_Evt_Sig_Beg,                        // Inclusive
    k_Seq_Evt_Sig_Min = k_Seq_Evt_Sig_Beg,    // Inclusive
    k_Seq_Evt_Sig_1st = k_Seq_Evt_Sig_Beg,    // Inclusive

    k_Seq_Evt_Sig_Instance_Initialized = k_Seq_Evt_Sig_Beg,
    #if CONFIG_FKMG_SEQ_SHUTDOWN_ENABLED
    k_Seq_Evt_Sig_Instance_Deinitialized,
    #endif
    k_Seq_Evt_Sig_Step_Occurred,
    k_Seq_Evt_Sig_Pot_Value_Changed,
    k_Seq_Evt_Sig_Btn_Value_Changed,
    k_Seq_Evt_Sig_LED_Write_Ready,
    k_Seq_Evt_Sig_MIDI_Write_Ready,
    k_Seq_Evt_Sig_End,                        // Exclusive
    k_Seq_Evt_Sig_Max = k_Seq_Evt_Sig_End - 1,// Inclusive
    k_Seq_Evt_Sig_Lst = k_Seq_Evt_Sig_End - 1,// Inclusive
    k_Seq_Evt_Sig_Cnt = k_Seq_Evt_Sig_End
                      - k_Seq_Evt_Sig_Beg,	  // Inclusive
	k_Seq_Evt_Sig_Ivd = k_Seq_Evt_Sig_End
};

/* Forward references to prevent include interdependent items getting declared
 * out-of-order. */
struct Sequencer_Instance;

/* Data signal k_Seq_Evt_Sig_Instance_Initialized can generate. */
struct Sequencer_Evt_Data_Instance_Initialized{
    struct Sequencer_Instance * p_inst;
};

/* Data signal k_Seq_Evt_Sig_Step_Occurred can generate. */
struct Sequencer_Evt_Step_Occurred{
    enum Sequencer_Id id;
};

/* Data signal k_Seq_Evt_Sig_Pot_Value_Changed can generate*/
struct Sequencer_Evt_Pot_Value_Changed{
    enum Sequencer_Id id;
    uint32_t val; 
};


/* Data signal k_Seq_Evt_Sig_Btn_Value_Changed can generate*/
struct Sequencer_Evt_Btn_Value_Changed{
    // enum Btn_Id id;
    bool val; 
};


/* Data signal k_Seq_Evt_Sig_LED_Write_Ready can generate*/
struct Sequencer_Evt_LED_Write_Ready{
    enum Sequencer_Id id;
    uint16_t val; 
};

/* Data signal k_Seq_Evt_Sig_MIDI_Write_Ready can generate*/
struct Sequencer_Evt_MIDI_Write_Ready{
    enum Sequencer_Id id;
    uint8_t midi_status; 
    uint16_t raw_voltage; 
    uint8_t last_note;
    uint8_t ctrl_byte;
};

/* Events (i.e. signal + signal's data if any) that can be generated. */
struct Sequencer_Evt{
	enum Sequencer_Evt_Sig sig;
	union Sequencer_Evt_Data{
        struct Sequencer_Evt_Data_Instance_Initialized  initd;
        struct Sequencer_Evt_Step_Occurred              stepped;
        struct Sequencer_Evt_Pot_Value_Changed          pot_changed;
        struct Sequencer_Evt_Btn_Value_Changed          btn_changed;
        struct Sequencer_Evt_LED_Write_Ready            led_write;
        struct Sequencer_Evt_MIDI_Write_Ready           midi_write;

	}data;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQ_EVT_H */
