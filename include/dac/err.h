/** ****************************************************************************
 * @brief DAC errors.
 *
 * The errors dac module generates.
 */

#ifndef FKMG_DAC_ERR_H
#define FKMG_DAC_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum DAC_Err_Id{
    k_DAC_Err_Id_Beg,                   // Inclusive
    k_DAC_Err_Id_Min = k_DAC_Err_Id_Beg,// Inclusive
    k_DAC_Err_Id_1st = k_DAC_Err_Id_Beg,// Inclusive

    k_DAC_Err_Id_None = k_DAC_Err_Id_Beg,
    k_DAC_Err_Id_Unknown,
    k_DAC_Err_Id_Unimplemented,
    k_DAC_Err_Id_Ptr_Invalid,
    k_DAC_Err_Id_Configuration_Invalid,

    k_DAC_Err_Id_End,                        // Exclusive
    k_DAC_Err_Id_Max = k_DAC_Err_Id_End - 1, // Inclusive
    k_DAC_Err_Id_Lst = k_DAC_Err_Id_End - 1, // Inclusive
    k_DAC_Err_Id_Cnt = k_DAC_Err_Id_End
                     - k_DAC_Err_Id_Beg,     // Inclusive
    k_DAC_Err_Id_Ivd = k_DAC_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_ERR_H */
