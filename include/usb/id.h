/** ****************************************************************************
 * @brief USB id.
 */

#ifndef FKMG_USB_ID_H
#define FKMG_USB_ID_H

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
enum USB_Id{
    k_USB_Id_Beg,                   // Inclusive
    k_USB_Id_Min = k_USB_Id_Beg,    // Inclusive
    k_USB_Id_1st = k_USB_Id_Beg,    // Inclusive

    /* TODO: */
    k_USB_Id_1 = k_USB_Id_Beg,      // Suggest using pcb silkscreen name or
                                    // similar common reference
    k_USB_Id_descriptive_name_1 = k_USB_Id_1, // Use descriptive name, like
                                               // front panel name
    /* etc */

    k_USB_Id_End,                   // Exclusive
    k_USB_Id_Max = k_USB_Id_End - 1,// Inclusive
    k_USB_Id_Lst = k_USB_Id_End - 1,// Inclusive
    k_USB_Id_Cnt = k_USB_Id_End
                 - k_USB_Id_Beg,	// Inclusive
	k_USB_Id_Ivd = k_USB_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_ID_H */
