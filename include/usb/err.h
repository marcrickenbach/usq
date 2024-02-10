/** ****************************************************************************
 * @brief USB errors.
 *
 * The errors USB module generates.
 */

#ifndef FKMG_USB_ERR_H
#define FKMG_USB_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum USB_Err_Id{
    k_USB_Err_Id_Beg,                   // Inclusive
    k_USB_Err_Id_Min = k_USB_Err_Id_Beg,// Inclusive
    k_USB_Err_Id_1st = k_USB_Err_Id_Beg,// Inclusive

    k_USB_Err_Id_None = k_USB_Err_Id_Beg,
    k_USB_Err_Id_Unknown,
    k_USB_Err_Id_Unimplemented,
    k_USB_Err_Id_Ptr_Invalid,
    k_USB_Err_Id_Configuration_Invalid,

    k_USB_Err_Id_End,                        // Exclusive
    k_USB_Err_Id_Max = k_USB_Err_Id_End - 1, // Inclusive
    k_USB_Err_Id_Lst = k_USB_Err_Id_End - 1, // Inclusive
    k_USB_Err_Id_Cnt = k_USB_Err_Id_End
                     - k_USB_Err_Id_Beg,     // Inclusive
    k_USB_Err_Id_Ivd = k_USB_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_USB_ERR_H */
