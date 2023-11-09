/** ****************************************************************************
 * @brief UART listener definition.
 */

#ifndef FKMG_UART_LISTENER_CB_H
#define FKMG_UART_LISTENER_CB_H

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
struct UART_Evt;

typedef void (*UART_Listener_Cb)(struct UART_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_UART_LISTENER_CB_H */
