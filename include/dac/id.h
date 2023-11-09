/** ****************************************************************************
 * @brief DAC id.
 */

#ifndef FKMG_DAC_ID_H
#define FKMG_DAC_ID_H

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
enum DAC_Id{
    k_DAC_Id_Beg,                   // Inclusive
    k_DAC_Id_Min = k_DAC_Id_Beg,    // Inclusive
    k_DAC_Id_1st = k_DAC_Id_Beg,    // Inclusive

    /* TODO: */
    k_DAC_Id_1 = k_DAC_Id_Beg,      // Suggest using pcb silkscreen name or
                                    // similar common reference
     k_DAC_Id_descriptive_name_1 = k_DAC_Id_1, // Use descriptive name, like
                                               // front panel name
    k_DAC_Id_2,
    k_DAC_Id_descriptive_name_2 = k_DAC_Id_2,
    k_DAC_Id_3,
    k_DAC_Id_descriptive_name_3 = k_DAC_Id_3,
    /* etc */

    k_DAC_Id_End,                   // Exclusive
    k_DAC_Id_Max = k_DAC_Id_End - 1,// Inclusive
    k_DAC_Id_Lst = k_DAC_Id_End - 1,// Inclusive
    k_DAC_Id_Cnt = k_DAC_Id_End
                 - k_DAC_Id_Beg,	// Inclusive
	k_DAC_Id_Ivd = k_DAC_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_ID_H */
