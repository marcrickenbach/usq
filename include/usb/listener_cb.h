/** ****************************************************************************
 * @brief USB listener definition.
 */

#ifndef FKMG_USB_LISTENER_CB_H
#define FKMG_USB_LISTENER_CB_H

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
struct USB_Evt;

typedef void (*USB_Listener_Cb)(struct USB_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_USB_LISTENER_CB_H */
