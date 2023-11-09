/** ****************************************************************************
 * @brief Pot listener definition.
 */

#ifndef FKMG_POT_LISTENER_CB_H
#define FKMG_POT_LISTENER_CB_H

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
struct Pot_Evt;

typedef void (*Pot_Listener_Cb)(struct Pot_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_POT_LISTENER_CB_H */
