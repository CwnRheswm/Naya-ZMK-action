# zmk-config-naya-right

Standalone ZMK firmware for a Naya Create **right half** whose factory firmware
cannot link to the left half. Built on ZMK v0.3 with the `nice_nano_v2` board
(same nRF52840 + Adafruit UF2 bootloader layout) plus a `naya_right` shield
whose matrix was recovered from the original firmware.

Build: push this repo to GitHub -> Actions -> download the `firmware` artifact
-> `naya_right-nice_nano_v2-zmk.uf2`.

Flash: put the right half in bootloader mode (NAYA Dongle drive appears) and
copy the .uf2 onto it.

Recovery: in this test build, hold any key for 3 s to get back to the
bootloader; copy your CURRENT.UF2 backup onto the drive to restore the
original firmware.
