/** ****************************************************************************
 * @brief Pot errors.
 *
 * The errors pot module generates.
 */

#ifndef FKMG_POT_ERR_H
#define FKMG_POT_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum Pot_Err_Id{
    k_Pot_Err_Id_Beg,                   // Inclusive
    k_Pot_Err_Id_Min = k_Pot_Err_Id_Beg,// Inclusive
    k_Pot_Err_Id_1st = k_Pot_Err_Id_Beg,// Inclusive

    k_Pot_Err_Id_None = k_Pot_Err_Id_Beg,
    k_Pot_Err_Id_Unknown,
    k_Pot_Err_Id_Unimplemented,
    k_Pot_Err_Id_Ptr_Invalid,
    k_Pot_Err_Id_Configuration_Invalid,

    k_Pot_Err_Id_End,                        // Exclusive
    k_Pot_Err_Id_Max = k_Pot_Err_Id_End - 1, // Inclusive
    k_Pot_Err_Id_Lst = k_Pot_Err_Id_End - 1, // Inclusive
    k_Pot_Err_Id_Cnt = k_Pot_Err_Id_End
                     - k_Pot_Err_Id_Beg,     // Inclusive
    k_Pot_Err_Id_Ivd = k_Pot_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_POT_ERR_H */
