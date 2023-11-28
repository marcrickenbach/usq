/** ****************************************************************************
 * @brief BUTTON listener definition.
 */

#ifndef FKMG_BUTTON_LISTENER_CB_H
#define FKMG_BUTTON_LISTENER_CB_H

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
struct Button_Evt;

typedef void (*Button_Listener_Cb)(struct Button_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_BUTTON_LISTENER_CB_H */
