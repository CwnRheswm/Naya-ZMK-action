#pragma once
#include <stdbool.h>
#include <stdint.h>

void naya_ovl_set_enabled(bool on);
bool naya_ovl_enabled(void);
int naya_ovl_set_slot_led(int slot, int led);
int naya_ovl_slot_led(int slot);
void naya_ovl_set_level(uint8_t level);
uint8_t naya_ovl_level(void);
void naya_ovl_test(int led);
int naya_ovl_test_led(void);
