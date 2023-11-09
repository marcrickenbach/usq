/** ****************************************************************************
 * @brief Pot id.
 */

#ifndef FKMG_POT_ID_H
#define FKMG_POT_ID_H

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
enum Pot_Id{
    k_Pot_Id_Beg,                   // Inclusive
    k_Pot_Id_Min = k_Pot_Id_Beg,    // Inclusive
    k_Pot_Id_1st = k_Pot_Id_Beg,    // Inclusive

    POT_RV_23 = k_Pot_Id_Beg,      // Suggest using pcb silkscreen name or similar common reference
     VOLTAGE_1 = POT_RV_23, // Use descriptive name, like front panel name
    POT_RV_4,
     VOLTAGE_2 = POT_RV_4,
    POT_RV_13,
     VOLTAGE_3 = POT_RV_13,
    POT_RV_22,
     VOLTAGE_4 = POT_RV_22,
    POT_RV_24,
     VOLTAGE_5 = POT_RV_24,
    POT_RV_26,
     VOLTAGE_6 = POT_RV_26,
    POT_RV_20,
     VOLTAGE_7 = POT_RV_20,
    POT_RV_15,
     VOLTAGE_8 = POT_RV_15,
    POT_RV_7,
     VOLTAGE_9 = POT_RV_7,
    POT_RV_33,
     VOLTAGE_10 = POT_RV_33,
    POT_RV_35,
     VOLTAGE_11 = POT_RV_35,
    POT_RV_30,
     VOLTAGE_12 = POT_RV_30,
    POT_RV_17,
     VOLTAGE_13 = POT_RV_17,
    POT_RV_18,
     VOLTAGE_14 = POT_RV_18,
    POT_RV_3,
     VOLTAGE_15 = POT_RV_3,
    POT_RV_6,
     VOLTAGE_16 = POT_RV_6,
    POT_RV_32,
     TIME_1 = POT_RV_32,
    POT_RV_1,
     TIME_2 = POT_RV_1,
    POT_RV_10,
     TIME_3 = POT_RV_10,
    POT_RV_2,
     TIME_4 = POT_RV_2,
    POT_RV_16,
     TIME_5 = POT_RV_16,
    POT_RV_28,
     TIME_6 = POT_RV_28,
    POT_RV_19,
     TIME_7 = POT_RV_19,
    POT_RV_31,
     TIME_8 = POT_RV_31,
    POT_RV_11,
     TIME_9 = POT_RV_11,
    POT_RV_12,
     TIME_10 = POT_RV_12,
    POT_RV_34,
     TIME_11 = POT_RV_34,
    POT_RV_8,
     TIME_12 = POT_RV_8,
    POT_RV_25,
     TIME_13 = POT_RV_25,
    POT_RV_29,
     TIME_14 = POT_RV_29,
    POT_RV_27,
     TIME_15 = POT_RV_27,
    POT_RV_5,
     TIME_16 = POT_RV_5,
    POT_RV_9,
     PARAM_1 = POT_RV_9,
    POT_RV_14,
     PARAM_2 = POT_RV_14,
    POT_RV_21,
    GLOBAL_POT = POT_RV_21,
    k_Pot_Id_End,                   // Exclusive
    k_Pot_Id_Max = k_Pot_Id_End - 1,// Inclusive
    k_Pot_Id_Lst = k_Pot_Id_End - 1,// Inclusive
    k_Pot_Id_Cnt = k_Pot_Id_End
                 - k_Pot_Id_Beg,	// Inclusive
	k_Pot_Id_Ivd = k_Pot_Id_End
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_POT_ID_H */
