/** ****************************************************************************
 * @brief UART errors.
 *
 * The errors UART module generates.
 */

#ifndef FKMG_UART_ERR_H
#define FKMG_UART_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum UART_Err_Id{
    k_UART_Err_Id_Beg,                   // Inclusive
    k_UART_Err_Id_Min = k_UART_Err_Id_Beg,// Inclusive
    k_UART_Err_Id_1st = k_UART_Err_Id_Beg,// Inclusive

    k_UART_Err_Id_None = k_UART_Err_Id_Beg,
    k_UART_Err_Id_Unknown,
    k_UART_Err_Id_Unimplemented,
    k_UART_Err_Id_Ptr_Invalid,
    k_UART_Err_Id_Configuration_Invalid,

    k_UART_Err_Id_End,                        // Exclusive
    k_UART_Err_Id_Max = k_UART_Err_Id_End - 1, // Inclusive
    k_UART_Err_Id_Lst = k_UART_Err_Id_End - 1, // Inclusive
    k_UART_Err_Id_Cnt = k_UART_Err_Id_End
                     - k_UART_Err_Id_Beg,     // Inclusive
    k_UART_Err_Id_Ivd = k_UART_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_UART_ERR_H */
