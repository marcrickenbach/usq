/** ****************************************************************************
 * @brief Seq Step id.
 */

#ifndef FKMG_SEQ_STEP_ID_H
#define FKMG_SEQ_STEP_ID_H

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
enum Sequencer_Step_Id{
    k_Seq_Step_Id_Beg,                   // Inclusive
    k_Seq_Step_Id_Min = k_Seq_Step_Id_Beg,    // Inclusive
    k_Seq_Step_Id_1st = k_Seq_Step_Id_Beg,    // Inclusive

    /* TODO: */
    k_Seq_Step_Id_0 = k_Seq_Step_Id_Beg,      // Suggest using pcb silkscreen name or
    k_Seq_Step_Id_1,
    k_Seq_Step_Id_2,
    k_Seq_Step_Id_3,
    k_Seq_Step_Id_4,
    k_Seq_Step_Id_5,
    k_Seq_Step_Id_6,
    k_Seq_Step_Id_7,
    k_Seq_Step_Id_8,
    k_Seq_Step_Id_9,
    k_Seq_Step_Id_10,
    k_Seq_Step_Id_11,
    k_Seq_Step_Id_12,
    k_Seq_Step_Id_13,
    k_Seq_Step_Id_14,
    k_Seq_Step_Id_15,
    k_Seq_Step_Id_End,     // Exclusive
    k_Seq_Step_Id_Max = k_Seq_Step_Id_End - 1,// Inclusive
    k_Seq_Step_Id_Lst = k_Seq_Step_Id_End - 1,// Inclusive
    k_Seq_Step_Id_Cnt = k_Seq_Step_Id_End
                 - k_Seq_Step_Id_Beg,	// Inclusive
	k_Seq_Step_Id_Ivd = k_Seq_Step_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQ_STEP_ID_H */
