#pragma once

// A single addressable LED used as the only feedback a headless board has: the
// preset number blinks out in white when the preset changes.

#ifdef ESP_PLATFORM
void status_led_start();
// Blinks `count` times, slowly. Requests replace each other rather than queue:
// the preset the player just chose is the one worth showing.
void status_led_blink(unsigned count);
#else
inline void status_led_start() {}
inline void status_led_blink(unsigned) {}
#endif
