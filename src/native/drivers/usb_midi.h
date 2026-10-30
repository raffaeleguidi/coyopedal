#pragma once

// USB MIDI host: a second USB host client that claims the interface's
// MIDIStreaming interface (class 1, subclass 3) and turns the foot switchers'
// messages into preset recalls and effect toggles.

#include <cstdint>

#ifdef ESP_PLATFORM
// Creates the client and its worker. Safe to call once, after the host library
// has been installed by the audio client.
void usb_midi_start();

// What the last received message looked like, and what the pedal did with it.
// Read from the low-priority audio heartbeat; null until one has arrived.
const char* usb_midi_last_message();
const char* usb_midi_mode_name();
std::uint32_t usb_midi_message_count();
#else
inline void usb_midi_start() {}
inline const char* usb_midi_last_message() {
    return "";
}
inline const char* usb_midi_mode_name() {
    return "";
}
inline std::uint32_t usb_midi_message_count() {
    return 0;
}
#endif
