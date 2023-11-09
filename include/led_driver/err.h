/** ****************************************************************************
 * @brief LED Driver errors.
 *
 * The errors Sequencer module generates.
 */

#ifndef FKMG_LED_DRIVER_ERR_H
#define FKMG_LED_DRIVER_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum LED_Driver_Err_Id{
    k_LED_Driver_Err_Id_Beg,                   // Inclusive
    k_LED_Driver_Err_Id_Min = k_LED_Driver_Err_Id_Beg,// Inclusive
    k_LED_Driver_Err_Id_1st = k_LED_Driver_Err_Id_Beg,// Inclusive

    k_LED_Driver_Err_Id_None = k_LED_Driver_Err_Id_Beg,
    k_LED_Driver_Err_Id_Unknown,
    k_LED_Driver_Err_Id_Unimplemented,
    k_LED_Driver_Err_Id_Ptr_Invalid,
    k_LED_Driver_Err_Id_Configuration_Invalid,

    k_LED_Driver_Err_Id_End,                        // Exclusive
    k_LED_Driver_Err_Id_Max = k_LED_Driver_Err_Id_End - 1, // Inclusive
    k_LED_Driver_Err_Id_Lst = k_LED_Driver_Err_Id_End - 1, // Inclusive
    k_LED_Driver_Err_Id_Cnt = k_LED_Driver_Err_Id_End
                     - k_LED_Driver_Err_Id_Beg,     // Inclusive
    k_LED_Driver_Err_Id_Ivd = k_LED_Driver_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_LED_DRIVER_ERR_H */
