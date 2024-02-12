/** ****************************************************************************
 * @brief LED Driver id.
 */

#ifndef FKMG_LED_DRIVER_ID_H
#define FKMG_LED_DRIVER_ID_H

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
enum LED_Driver_Id{
    k_LED_Driver_Id_Beg,                   // Inclusive
    k_LED_Driver_Id_Min = k_LED_Driver_Id_Beg,    // Inclusive
    k_LED_Driver_Id_1st = k_LED_Driver_Id_Beg,    // Inclusive

    /* TODO: */
    k_LED_Driver_Channel_0 = k_LED_Driver_Id_Beg,      // Suggest using pcb silkscreen name or
    k_LED_Driver_Channel_1,
    k_LED_Driver_Channel_2,
    k_LED_Driver_Channel_3,
    k_LED_Driver_Channel_4,
    k_LED_Driver_Channel_5,
    k_LED_Driver_Id_End,                   // Exclusive
    k_LED_Driver_Id_Max = k_LED_Driver_Id_End - 1,// Inclusive
    k_LED_Driver_Id_Lst = k_LED_Driver_Id_End - 1,// Inclusive
    k_LED_Driver_Id_Cnt = k_LED_Driver_Id_End
                 - k_LED_Driver_Id_Beg,	// Inclusive
	k_LED_Driver_Id_Ivd = k_LED_Driver_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_LED_DRIVER_ID_H */
