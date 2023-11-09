/** ****************************************************************************
 * @brief Sequencer errors.
 *
 * The errors Sequencer module generates.
 */

#ifndef FKMG_SEQ_ERR_H
#define FKMG_SEQ_ERR_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Enums
 */

/* Errors generated. */
enum Sequencer_Err_Id{
    k_Seq_Err_Id_Beg,                   // Inclusive
    k_Seq_Err_Id_Min = k_Seq_Err_Id_Beg,// Inclusive
    k_Seq_Err_Id_1st = k_Seq_Err_Id_Beg,// Inclusive

    k_Seq_Err_Id_None = k_Seq_Err_Id_Beg,
    k_Seq_Err_Id_Unknown,
    k_Seq_Err_Id_Unimplemented,
    k_Seq_Err_Id_Ptr_Invalid,
    k_Seq_Err_Id_Configuration_Invalid,

    k_Seq_Err_Id_End,                        // Exclusive
    k_Seq_Err_Id_Max = k_Seq_Err_Id_End - 1, // Inclusive
    k_Seq_Err_Id_Lst = k_Seq_Err_Id_End - 1, // Inclusive
    k_Seq_Err_Id_Cnt = k_Seq_Err_Id_End
                     - k_Seq_Err_Id_Beg,     // Inclusive
    k_Seq_Err_Id_Ivd = k_Seq_Err_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQ_ERR_H */
