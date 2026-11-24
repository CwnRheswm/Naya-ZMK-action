/*
 * Bluetooth slot light.
 *
 * Sits between ZMK's RGB underglow and the real WS2812 strip (chosen zmk,underglow points here).
 * While the keyboard types over Bluetooth, the LED under the active slot's key is drawn white on
 * top of whatever effect is running: solid when that slot is connected, blinking while it waits
 * for its computer. Over USB, or with the feature off, frames pass through unchanged.
 *
 * Settings (saved on the keyboard, set from the Naya Control app via naya_ctrl.c "ovl" commands):
 *   enabled, LED index for each of the 5 slots (-1 = none), white level.
 */
#define DT_DRV_COMPAT naya_led_overlay

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/settings/settings.h>
#include <string.h>
#include <errno.h>

#include <zmk/rgb_underglow.h>
#include <zmk/endpoints.h>
#include <zmk/ble.h>

#include "led_overlay.h"

#define OVL_MAX_PIXELS 64
#define OVL_SLOTS 5

struct ovl_settings {
    uint8_t enabled;
    uint8_t level;
    int8_t slot_led[OVL_SLOTS];
};

static struct ovl_settings cfg = {
    .enabled = 1,
    .level = 160,
    .slot_led = {-1, -1, -1, -1, -1},
};
static int test_led = -1;
static struct led_rgb frame[OVL_MAX_PIXELS];

struct ovl_config {
    const struct device *strip;
};

static void paint(struct led_rgb *px, size_t n, int idx) {
    if (idx >= 0 && (size_t)idx < n) {
        px[idx].r = cfg.level;
        px[idx].g = cfg.level;
        px[idx].b = cfg.level;
    }
}

static void apply(struct led_rgb *px, size_t n) {
    bool on = false;
    zmk_rgb_underglow_get_state(&on);
    if (!on) {
        return; /* LEDs switched off (or timed out): keep them dark */
    }
    if (test_led >= 0) { /* LED finder: only the test LED is lit */
        for (size_t i = 0; i < n; i++) {
            px[i].r = px[i].g = px[i].b = 0;
        }
        paint(px, n, test_led);
        return;
    }
    if (!cfg.enabled || zmk_endpoints_selected().transport != ZMK_TRANSPORT_BLE) {
        return;
    }
    int slot = zmk_ble_active_profile_index();
    if (slot < 0 || slot >= OVL_SLOTS) {
        return;
    }
    if (zmk_ble_active_profile_is_connected() || (k_uptime_get() / 400) % 2 == 0) {
        paint(px, n, cfg.slot_led[slot]);
    }
}

static int ovl_update_rgb(const struct device *dev, struct led_rgb *pixels, size_t num_pixels) {
    const struct ovl_config *c = dev->config;
    if (!device_is_ready(c->strip)) {
        return -ENODEV;
    }
    size_t n = MIN(num_pixels, (size_t)OVL_MAX_PIXELS);
    memcpy(frame, pixels, n * sizeof(struct led_rgb));
    apply(frame, n);
    return led_strip_update_rgb(c->strip, frame, n);
}

static int ovl_update_channels(const struct device *dev, uint8_t *channels, size_t num_channels) {
    return -ENOTSUP;
}

static const struct led_strip_driver_api ovl_api = {
    .update_rgb = ovl_update_rgb,
    .update_channels = ovl_update_channels,
};

static int ovl_init(const struct device *dev) { return 0; }

#define OVL_INIT(inst)                                                                             \
    static const struct ovl_config ovl_config_##inst = {                                           \
        .strip = DEVICE_DT_GET(DT_INST_PHANDLE(inst, led_strip)),                                  \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, ovl_init, NULL, NULL, &ovl_config_##inst, POST_KERNEL,             \
                          CONFIG_LED_STRIP_INIT_PRIORITY, &ovl_api);

DT_INST_FOREACH_STATUS_OKAY(OVL_INIT)

/* ---- settings ---- */
static int ovl_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    if (!strcmp(name, "cfg") && len == sizeof(cfg)) {
        struct ovl_settings tmp;
        if (read_cb(cb_arg, &tmp, sizeof(tmp)) == sizeof(tmp)) {
            cfg = tmp;
        }
    }
    return 0;
}
SETTINGS_STATIC_HANDLER_DEFINE(naya_ovl, "naya_ovl", NULL, ovl_settings_set, NULL, NULL);

static void save(void) { settings_save_one("naya_ovl/cfg", &cfg, sizeof(cfg)); }

/* ---- API for naya_ctrl.c ---- */
void naya_ovl_set_enabled(bool on) {
    cfg.enabled = on;
    save();
}
bool naya_ovl_enabled(void) { return cfg.enabled; }
int naya_ovl_set_slot_led(int slot, int led) {
    if (slot < 0 || slot >= OVL_SLOTS || led < -1 || led >= OVL_MAX_PIXELS) {
        return -EINVAL;
    }
    cfg.slot_led[slot] = (int8_t)led;
    save();
    return 0;
}
int naya_ovl_slot_led(int slot) { return (slot >= 0 && slot < OVL_SLOTS) ? cfg.slot_led[slot] : -1; }
void naya_ovl_set_level(uint8_t level) {
    cfg.level = level;
    save();
}
uint8_t naya_ovl_level(void) { return cfg.level; }
void naya_ovl_test(int led) { test_led = (led >= 0 && led < OVL_MAX_PIXELS) ? led : -1; }
int naya_ovl_test_led(void) { return test_led; }
