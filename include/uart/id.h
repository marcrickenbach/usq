/** ****************************************************************************
 * @brief UART id.
 */

#ifndef FKMG_UART_ID_H
#define FKMG_UART_ID_H

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
enum UART_Id{
    k_UART_Id_Beg,                   // Inclusive
    k_UART_Id_Min = k_UART_Id_Beg,    // Inclusive
    k_UART_Id_1st = k_UART_Id_Beg,    // Inclusive

    UART_1 = k_UART_Id_Beg,      // Currently set up assuming 16 midi channels
    UART_2,
    UART_3,
    UART_4,
    UART_5,
    UART_6,
    UART_7,
    UART_8,
    UART_9,
    UART_10,
    UART_11,
    UART_12,
    UART_13,
    UART_14,
    UART_15,
    UART_16,
    k_UART_Id_End,                   // Exclusive
    k_UART_Id_Max = k_UART_Id_End - 1,// Inclusive
    k_UART_Id_Lst = k_UART_Id_End - 1,// Inclusive
    k_UART_Id_Cnt = k_UART_Id_End
                 - k_UART_Id_Beg,	// Inclusive
	k_UART_Id_Ivd = k_UART_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_UART_ID_H */
