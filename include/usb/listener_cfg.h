/** ****************************************************************************
 * @brief USB listener configuration interface.
 */

#ifndef FKMG_USB_LISTENER_CB_CFG_H
#define FKMG_USB_LISTENER_CB_CFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* *****************************************************************************
 * Includes
 */

#include <zephyr/kernel.h>

#include "listener_cb.h"
#include "evt.h"

/* *****************************************************************************
 * Listener Callback Typedef
 */

/* Forward references to prevent include interdependent items getting declared
 * out-of-order. */
struct USB_Instance;
struct USB_Listener;

struct USB_Listener_Cfg{
    /* Required: pointer to initialized opaque instace we'll add listener to. */
    struct USB_Instance * p_inst;

    /* Required: pointer to uninitialized/unused opaque listener to config. */
    struct USB_Listener * p_lsnr;

    /* Required: event signal to listen for. */
    enum USB_Evt_Sig sig;

    /* Required: function to call back and send event to when signal occurs. */
    USB_Listener_Cb cb;
};

#ifdef __cplusplus
}
#endif

#endif /* FKMG_USB_LISTENER_CB_CFG_H */
