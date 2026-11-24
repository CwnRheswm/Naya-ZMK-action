# zmk-config-naya-right

Standalone ZMK firmware for a Naya Create **right half** whose factory firmware
cannot link to the left half. Built on ZMK v0.3 with the `nice_nano_v2` board
(same nRF52840 + Adafruit UF2 bootloader layout) plus a `naya_right` shield
whose matrix was recovered from the original firmware and mapped by key test.

Layers: 0 = Naya Default right half, 1 = F-keys/numpad (Naya layer 1),
2 = Bluetooth / USB-BLE output / media / bootloader.

Bootloader: hold MO2 (bottom-right key) and tap the top-left key; the
NAYA Dongle drive appears. Copy CURRENT.UF2 onto it to restore the factory firmware.

## Remapping
ZMK Studio is enabled. Open https://zmk.studio in Chrome or Edge with the right
half plugged in by USB, connect, and unlock with S4 + the 7th key of the N/M row.

## Naya Control app
`src/naya_ctrl.c` adds a second USB serial port with text commands
(`ver`, `bat`, `rgb on|off|toggle|state|hsb H S B|eff N|next|prev|spd +|-`, `boot`).
The Naya Control page uses it to read the battery and drive the LEDs.
