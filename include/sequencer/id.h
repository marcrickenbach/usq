/** ****************************************************************************
 * @brief Seq id.
 */

#ifndef FKMG_SEQ_ID_H
#define FKMG_SEQ_ID_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

/* *****************************************************************************
 * Events
 */

/* Signals that can be generated as part of an event. */
enum Sequencer_Id{
    k_Seq_Id_Beg,                   // Inclusive
    k_Seq_Id_Min = k_Seq_Id_Beg,    // Inclusive
    k_Seq_Id_1st = k_Seq_Id_Beg,    // Inclusive

    /* TODO: */
    k_Seq_Id_1 = k_Seq_Id_Beg,      // Suggest using pcb silkscreen name or
                                    // similar common reference
     k_Seq_Id_channel_1 = k_Seq_Id_1, // Use descriptive name, like
                                               // front panel name
    k_Seq_Id_2,
     k_Seq_Id_channel_2 = k_Seq_Id_2,

    k_Seq_Id_End,                   // Exclusive
    k_Seq_Id_Max = k_Seq_Id_End - 1,// Inclusive
    k_Seq_Id_Lst = k_Seq_Id_End - 1,// Inclusive
    k_Seq_Id_Cnt = k_Seq_Id_End
                 - k_Seq_Id_Beg,	// Inclusive
	k_Seq_Id_Ivd = k_Seq_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQ_ID_H */
