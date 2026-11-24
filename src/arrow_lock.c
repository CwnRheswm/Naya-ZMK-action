/*
 * Arrow layer follows Scroll Lock.
 *
 * The two halves are separate keyboards, so a key on the left half can't switch a layer here directly.
 * What both halves do share is the computer's lock-light state (Num/Caps/Scroll Lock), which the
 * computer sends to every keyboard. So: a left-half key sends Scroll Lock, and while Scroll Lock is on
 * this half turns on its Arrows layer.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/keymap.h>

LOG_MODULE_REGISTER(arrow_lock, CONFIG_ZMK_LOG_LEVEL);

#define ARROW_LAYER_INDEX 3
#define SCROLL_LOCK_BIT BIT(2) /* HID LED bits: 0 Num Lock, 1 Caps Lock, 2 Scroll Lock */

static int arrow_lock_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    zmk_keymap_layer_id_t id = zmk_keymap_layer_index_to_id(ARROW_LAYER_INDEX);
    if (ev->indicators & SCROLL_LOCK_BIT) {
        zmk_keymap_layer_activate(id);
    } else {
        zmk_keymap_layer_deactivate(id);
    }
    LOG_DBG("indicators 0x%02x -> arrow layer %s", ev->indicators,
            (ev->indicators & SCROLL_LOCK_BIT) ? "on" : "off");
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(arrow_lock, arrow_lock_listener);
ZMK_SUBSCRIPTION(arrow_lock, zmk_hid_indicators_changed);
