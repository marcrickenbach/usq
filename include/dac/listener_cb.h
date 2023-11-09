/** ****************************************************************************
 * @brief DAC listener definition.
 */

#ifndef FKMG_DAC_LISTENER_CB_H
#define FKMG_DAC_LISTENER_CB_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

/* *****************************************************************************
 * Listener Callback Typedef
 */

/* Forward references to prevent include interdependent items getting declared
 * out-of-order. */
struct DAC_Evt;

typedef void (*DAC_Listener_Cb)(struct DAC_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_DAC_LISTENER_CB_H */
