/** ****************************************************************************
 * @brief Button id.
 */

#ifndef FKMG_BUTTON_ID_H
#define FKMG_BUTTON_ID_H

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
enum Button_Id{
    k_Button_Id_Beg,                   // Inclusive
    k_Button_Id_Min = k_Button_Id_Beg,    // Inclusive
    k_Button_Id_1st = k_Button_Id_Beg,    // Inclusive

    /* TODO: */
    k_Button_Id_1 = k_Button_Id_Beg,      // Suggest using pcb silkscreen name or
                                    // similar common reference
    k_Button_Id_descriptive_name_1 = k_Button_Id_1, // Use descriptive name, like
                                               // front panel name
    k_Button_Id_2,
    k_Button_Id_descriptive_name_2 = k_Button_Id_2,
    k_Button_Id_3,
    k_Button_Id_descriptive_name_3 = k_Button_Id_3,
    /* etc */

    k_Button_Id_End,                   // Exclusive
    k_Button_Id_Max = k_Button_Id_End - 1,// Inclusive
    k_Button_Id_Lst = k_Button_Id_End - 1,// Inclusive
    k_Button_Id_Cnt = k_Button_Id_End
                 - k_Button_Id_Beg,	// Inclusive
	k_Button_Id_Ivd = k_Button_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_ID_H */
