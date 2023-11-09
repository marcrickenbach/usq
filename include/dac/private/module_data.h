/*******************************************************************************
 * @brief DAC module data interface.
 *
 * This is the private module data.
 */

#ifndef FKMG_DAC_MODULE_DATA_H
#define FKMG_DAC_MODULE_DATA_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

/* *****************************************************************************
 * Enums
 */

/* *****************************************************************************
 * Structs
 */

struct dac_module_data{
    bool initialized;

    /* Singly linked lists to keep track of things. */
    struct{
        sys_slist_t instances;
    }list;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_MODULE_DATA_H */
