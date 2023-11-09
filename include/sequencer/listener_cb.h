/** ****************************************************************************
 * @brief Seq listener definition.
 */

#ifndef FKMG_SEQ_LISTENER_CB_H
#define FKMG_SEQ_LISTENER_CB_H

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
struct Sequencer_Evt;

typedef void (*Sequencer_Listener_Cb)(struct Sequencer_Evt *p_evt);

#ifdef __cplusplus
}
#endif

#endif /* FKMG_SEQ_LISTENER_CB_H */
