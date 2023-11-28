/** ****************************************************************************
 * @brief BUTTON errors.
 *
 * The errors Button module generates.
 */

#ifndef FKMG_BUTTON_ERR_H
#define FKMG_BUTTON_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum Button_Err_Id{
    k_Button_Err_Id_Beg,                   // Inclusive
    k_Button_Err_Id_Min = k_Button_Err_Id_Beg,// Inclusive
    k_Button_Err_Id_1st = k_Button_Err_Id_Beg,// Inclusive

    k_Button_Err_Id_None = k_Button_Err_Id_Beg,
    k_Button_Err_Id_Unknown,
    k_Button_Err_Id_Unimplemented,
    k_Button_Err_Id_Ptr_Invalid,
    k_Button_Err_Id_Configuration_Invalid,

    k_Button_Err_Id_End,                        // Exclusive
    k_Button_Err_Id_Max = k_Button_Err_Id_End - 1, // Inclusive
    k_Button_Err_Id_Lst = k_Button_Err_Id_End - 1, // Inclusive
    k_Button_Err_Id_Cnt = k_Button_Err_Id_End
                     - k_Button_Err_Id_Beg,     // Inclusive
    k_Button_Err_Id_Ivd = k_Button_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_BUTTON_ERR_H */
