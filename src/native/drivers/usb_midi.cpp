// USB MIDI host for the foot switches.
//
// The audio client owns the streaming interfaces. This is a second, independent
// USB host client that claims only the MIDIStreaming interface (class 1,
// subclass 3) and reads its IN endpoint, so a MIDI device and an audio device
// can be the same box without either client seeing the other's traffic.
//
// It runs at a low priority on purpose. A switch press is a human gesture, and
// delaying one by a block or two is inaudible; delaying the audio client's
// isochronous completions is not, and those share the same host library.
//
// Nothing here touches the engine from the USB callback. A preset recall reads
// the preset partition and writes the DSP state; run from the client's event
// loop it would stall the completions that share it, and from a PSRAM stack it
// would run with the flash cache disabled. So the callback only enqueues, and a
// worker with an internal stack performs the change.
//
// The switch numbers come from the XTONE manual's mode table, not from a guess.
// The XTONE keeps three groups and sends a different control change per group,
// so the group is visible in the number itself and the pedal needs no mode of
// its own:
//
//   group A (green)  FS A=10  FS B=20  FS C=11   EXP=7
//   group B (blue)   FS A=22  FS B=24  FS C=25   EXP=4
//   group C (red)    FS A=30  FS B=43  FS C=42   EXP=1
//
// Green selects the three presets; red toggles one effect per switch. Blue is
// left unmapped on purpose: it is the group a player can keep free.

#include "usb_midi.h"
#include "usb_audio.h"

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "audio/effects.h"
#include "control.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"

namespace {

constexpr const char* kTag = "usb_midi";

// USB audio class 1 (the class the MIDIStreaming subclass belongs to) and its
// MIDIStreaming subclass. The MIDI interface is claimed on its own; the audio
// streaming interfaces are claimed by usb_audio.cpp's client.
constexpr std::uint8_t kMidiStreamingSubclass = 0x03;

constexpr std::uint8_t kControlChange = 0xB0;
constexpr std::uint8_t kProgramChange = 0xC0;

// A switch press is a value near 127; the release is 0. Acting on the release
// as well would undo the press, so only values in the upper half count.
constexpr std::uint8_t kPressThreshold = 64U;

constexpr std::uint8_t kSwitchCount = 3U;
constexpr std::uint32_t kMidiTaskStackBytes = 4096U;
constexpr std::uint32_t kSetupTaskStackBytes = 4096U;
constexpr std::uint32_t kWorkerStackBytes = 4096U;
constexpr UBaseType_t kMidiTaskPriority = 10U;
constexpr UBaseType_t kSetupTaskPriority = 9U;
constexpr UBaseType_t kWorkerPriority = 8U;
constexpr BaseType_t kMidiCore = 0;

// How long the claim keeps retrying while the audio client is still taking its
// own interfaces. The controller's DMA descriptors come out of internal SRAM,
// and at the moment the device enumerates the largest free internal block can be
// smaller than one descriptor; a second later it is not. Twenty attempts of
// 500 ms covers the whole bring-up with room to spare.
constexpr int kClaimAttempts = 20;
constexpr std::uint32_t kClaimRetryDelayMs = 500U;

struct Action {
    enum Kind : std::uint8_t { LoadPreset, ToggleBlock } kind;
    std::uint8_t index;
};

// What one control change does. The number is the XTONE's own, from the manual's
// group table above; a pedal that grew a different interface only replaces this
// table.
struct Binding {
    std::uint8_t cc;
    Action::Kind kind;
    std::uint8_t index;
};

constexpr Binding kBindings[] = {
    // Group A (green): the three switches recall the three factory presets.
    {10U, Action::LoadPreset, 0U},
    {20U, Action::LoadPreset, 1U},
    {11U, Action::LoadPreset, 2U},
    // Group C (red): the three switches toggle one block each.
    {30U, Action::ToggleBlock, COYOPEDAL_FX_OVERDRIVE},
    {43U, Action::ToggleBlock, COYOPEDAL_FX_MODULATION},
    {42U, Action::ToggleBlock, COYOPEDAL_FX_DELAY},
};

QueueHandle_t actions{};
QueueHandle_t setupQueue{};

usb_host_client_handle_t client{};
usb_device_handle_t device{};        // opened and claimed; transfers run here
usb_device_handle_t pendingDevice{}; // opened, waiting for a claim
usb_transfer_t* transfer{};
std::uint8_t claimedInterface = 0xFFU;
bool deviceOpen{};

// Diagnostics for the heartbeat: the last message seen, how many, and which
// XTONE group it came from.
portMUX_TYPE messageLock = portMUX_INITIALIZER_UNLOCKED;
char lastMessageText[48] = "none";
std::atomic<std::uint32_t> messageCount{0U};
// True while an IN transfer is in flight. A device that never sends anything and
// a controller that silently drops oversized packets look identical from the
// counters alone; this is what tells them apart in the heartbeat.
std::atomic<bool> transferPending{false};
std::atomic<const char*> lastGroup{"none"};

const char* group_name(const std::uint8_t cc) {
    if (cc == 10U || cc == 20U || cc == 11U || cc == 7U) {
        return "green";
    }
    if (cc == 22U || cc == 24U || cc == 25U || cc == 4U) {
        return "blue";
    }
    if (cc == 30U || cc == 43U || cc == 42U || cc == 1U) {
        return "red";
    }
    return "none";
}

void note_message(const std::uint8_t code, const std::uint8_t status, const std::uint8_t data1,
                  const std::uint8_t data2) {
    char text[48];
    std::snprintf(text, sizeof text, "cin=%x ch=%u status=%02x d1=%u d2=%u", code,
                  static_cast<unsigned>(status & 0x0FU) + 1U, status, data1, data2);
    taskENTER_CRITICAL(&messageLock);
    std::memcpy(lastMessageText, text, sizeof lastMessageText);
    taskEXIT_CRITICAL(&messageLock);
    messageCount.fetch_add(1U, std::memory_order_relaxed);
    ESP_LOGI(kTag, "midi %s (group %s)", text, group_name(data1));
}

void enqueue(const Action& action) {
    if (actions == nullptr) {
        return;
    }
    if (xQueueSend(actions, &action, 0) != pdTRUE) {
        ESP_LOGW(kTag, "MIDI action queue full; dropped kind=%u index=%u",
                 static_cast<unsigned>(action.kind), static_cast<unsigned>(action.index));
    }
}

void dispatch(const std::uint8_t status, const std::uint8_t data1, const std::uint8_t data2) {
    const std::uint8_t kind = status & 0xF0U;
    // A program change names a preset outright; an interface configured that way
    // needs no control-change table at all.
    if (kind == kProgramChange) {
        if (data1 < kSwitchCount) {
            enqueue({Action::LoadPreset, data1});
        }
        return;
    }
    if (kind != kControlChange || data2 < kPressThreshold) {
        return;
    }
    lastGroup.store(group_name(data1));
    for (const Binding& binding : kBindings) {
        if (binding.cc != data1) {
            continue;
        }
        enqueue({binding.kind, binding.index});
        return;
    }
}

void handle_packets(const std::uint8_t* const data, const int bytes) {
    // USB-MIDI event packets are always four bytes: a cable/CIN header and three
    // data bytes. A transfer can carry several.
    for (int offset = 0; offset + 3 < bytes; offset += 4) {
        const std::uint8_t code = data[offset] & 0x0FU;
        if (code == 0x0U) {
            continue;
        }
        note_message(code, data[offset + 1], data[offset + 2], data[offset + 3]);
        dispatch(data[offset + 1], data[offset + 2], data[offset + 3]);
    }
}

void release_device(usb_device_handle_t handle) {
    if (handle == nullptr) {
        return;
    }
    if (claimedInterface != 0xFFU && client != nullptr) {
        usb_host_interface_release(client, handle, claimedInterface);
        claimedInterface = 0xFFU;
    }
    if (client != nullptr) {
        usb_host_device_close(client, handle);
    }
    deviceOpen = false;
}

void submit_transfer() {
    if (device == nullptr || transfer == nullptr) {
        return;
    }
    transfer->num_bytes = transfer->data_buffer_size;
    transferPending.store(true, std::memory_order_relaxed);
    const esp_err_t result = usb_host_transfer_submit(transfer);
    if (result != ESP_OK) {
        transferPending.store(false, std::memory_order_relaxed);
        ESP_LOGW(kTag, "MIDI transfer submit failed: %s", esp_err_to_name(result));
    }
}

void transfer_callback(usb_transfer_t* const completed) {
    if (completed == nullptr) {
        return;
    }
    transferPending.store(false, std::memory_order_relaxed);
    if (completed->status == USB_TRANSFER_STATUS_COMPLETED) {
        handle_packets(completed->data_buffer, completed->actual_num_bytes);
        submit_transfer();
        return;
    }
    if (completed->status == USB_TRANSFER_STATUS_NO_DEVICE ||
        completed->status == USB_TRANSFER_STATUS_CANCELED) {
        // The device is gone. Freeing is safe here: this callback runs after the
        // transfer has completed, and device is already null so nothing
        // resubmitted.
        const usb_device_handle_t gone = device;
        device = nullptr;
        usb_host_transfer_free(completed);
        transfer = nullptr;
        release_device(gone);
        return;
    }
    // The endpoint was halted: an overflow means the device put more on the wire
    // than the transfer asked for. Clearing and resubmitting in place fed the
    // controller a transfer it had already completed and tripped
    // hcd_dwc.c's parse assert, which costs the whole boot. Losing the switches
    // is the cheaper failure, so this ends the midi stream for this boot and the
    // audio keeps running.
    ESP_LOGW(kTag, "MIDI transfer status %d; endpoint 0x%02x halted, switches off for this boot",
             static_cast<int>(completed->status), completed->bEndpointAddress);
    if (device != nullptr) {
        usb_host_endpoint_clear(device, completed->bEndpointAddress);
    }
}

struct MidiEndpoint {
    std::uint8_t interfaceNumber{0xFFU};
    std::uint8_t alternateSetting{0U};
    std::uint8_t address{0U};
    std::uint16_t mps{0U};
};

// Walks the active configuration looking for the MIDIStreaming interface and its
// IN endpoint. The interface is claimed exactly as the descriptor names it: an
// interface that offers its endpoints only on a non-zero alternate setting is
// claimed on that one.
bool find_midi_endpoint(usb_device_handle_t handle, MidiEndpoint& found) {
    const usb_config_desc_t* config = nullptr;
    if (usb_host_get_active_config_descriptor(handle, &config) != ESP_OK || config == nullptr) {
        return false;
    }
    const std::uint8_t* const base = reinterpret_cast<const std::uint8_t*>(config);
    const int total = config->wTotalLength;
    bool inMidi = false;
    for (int offset = 0; offset + 2 <= total;) {
        const std::uint8_t length = base[offset];
        const std::uint8_t type = base[offset + 1];
        if (length < 2U) {
            break;
        }
        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && length >= USB_INTF_DESC_SIZE) {
            const auto* const interface = reinterpret_cast<const usb_intf_desc_t*>(base + offset);
            inMidi = interface->bInterfaceClass == USB_CLASS_AUDIO &&
                     interface->bInterfaceSubClass == kMidiStreamingSubclass;
            // The whole layout is logged: which interface carries the MIDI
            // subclass is the device's choice, and a claim that fails needs the
            // numbers in front of it.
            ESP_LOGI(kTag, "intf %u alt %u class %02x/%02x eps %u", interface->bInterfaceNumber,
                     interface->bAlternateSetting, interface->bInterfaceClass,
                     interface->bInterfaceSubClass, interface->bNumEndpoints);
            if (inMidi) {
                found.interfaceNumber = interface->bInterfaceNumber;
                found.alternateSetting = interface->bAlternateSetting;
            }
        } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && length >= USB_EP_DESC_SIZE) {
            const auto* const endpoint = reinterpret_cast<const usb_ep_desc_t*>(base + offset);
            const std::uint8_t kind = endpoint->bmAttributes & 0x03U;
            ESP_LOGI(kTag, "  ep %02x type %u mps %u%s", endpoint->bEndpointAddress, kind,
                     static_cast<unsigned>(endpoint->wMaxPacketSize & 0x07FFU),
                     inMidi ? " (midi)" : "");
            if (inMidi && (endpoint->bEndpointAddress & 0x80U) != 0U &&
                (kind == USB_TRANSFER_TYPE_INTR || kind == USB_TRANSFER_TYPE_BULK)) {
                found.address = endpoint->bEndpointAddress;
                found.mps = static_cast<std::uint16_t>(endpoint->wMaxPacketSize & 0x07FFU);
                return found.mps > 0U;
            }
        }
        offset += length;
    }
    return false;
}

// The largest packet a Full-Speed bulk or interrupt endpoint may carry. USB 2.0
// fixes it at 64 bytes; 512 is the High-Speed size.
//
// The controller's own ceiling is what actually decides here, and it is not one
// number: the non-periodic OUT FIFO holds 16 words (64 bytes) while the shared
// RX FIFO holds 86 usable words (344 bytes). An interface claim programs both
// directions, so an OUT endpoint over 64 refuses the whole interface even though
// this client only reads.
constexpr std::uint16_t kFullSpeedOutPacketMax = 64U;
constexpr std::uint16_t kControllerRxPacketMax = 344U;

// Clamps the cached configuration descriptor's bulk and interrupt packet sizes.
//
// The XTONE reports one configuration, and its MIDIStreaming interface carries
// two bulk endpoints declared at 512 bytes -- the High-Speed figure -- even when
// it answers a Full-Speed host. The S3's controller refuses to program a bulk
// endpoint wider than 64 at Full Speed ("EP MPS (512) exceeds supported limit"),
// so the interface claim fails with ESP_ERR_NOT_SUPPORTED and no MIDI transfer is
// ever submitted.
//
// The number a USB 2.0 host may put on the wire at Full Speed is 64 regardless of
// what the device printed, so clamping is what the bus already enforces. It is
// applied to the host library's cached copy, which is the buffer the interface
// claim reads to program the controller. Isochronous endpoints -- the audio
// streams -- are left alone: they may legitimately exceed 64 and they belong to
// the other client.
void clamp_full_speed_packet_sizes(usb_device_handle_t handle) {
    const usb_config_desc_t* cached = nullptr;
    if (usb_host_get_active_config_descriptor(handle, &cached) != ESP_OK || cached == nullptr) {
        return;
    }
    auto* const base = reinterpret_cast<std::uint8_t*>(const_cast<usb_config_desc_t*>(cached));
    const int total = cached->wTotalLength;
    for (int offset = 0; offset + 2 <= total;) {
        const std::uint8_t length = base[offset];
        const std::uint8_t type = base[offset + 1];
        if (length < 2U) {
            break;
        }
        if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && length >= USB_EP_DESC_SIZE) {
            auto* const endpoint = reinterpret_cast<usb_ep_desc_t*>(base + offset);
            const std::uint8_t kind = endpoint->bmAttributes & 0x03U;
            const std::uint16_t mps =
                static_cast<std::uint16_t>(endpoint->wMaxPacketSize & 0x07FFU);
            // Each direction is clamped to its own ceiling: the OUT FIFO's 64
            // bytes for an endpoint this client never writes to anyway, and the
            // shared RX FIFO's 344 for the IN endpoint the switches arrive on.
            // 344 is not a legal Full-Speed packet size, but it is what the
            // controller can hold and the pipe only has to be at least as wide
            // as the packet the device actually sends.
            const bool is_in = (endpoint->bEndpointAddress & 0x80U) != 0U;
            const std::uint16_t ceiling = is_in ? kControllerRxPacketMax : kFullSpeedOutPacketMax;
            if ((kind == USB_TRANSFER_TYPE_BULK || kind == USB_TRANSFER_TYPE_INTR) &&
                mps > ceiling) {
                ESP_LOGW(kTag, "endpoint 0x%02x declares %u bytes; clamping to %u",
                         endpoint->bEndpointAddress, mps, ceiling);
                endpoint->wMaxPacketSize = ceiling;
            }
        }
        offset += length;
    }
}

// The XTONE answers enumeration with more than one configuration. Only the
// active one can be claimed, but reading the others says whether a Full-Speed
// shape exists at all -- one with 64-byte bulk endpoints -- which would be the
// configuration to switch to instead of clamping.
void dump_configuration(usb_device_handle_t handle, const std::uint8_t value) {
    const usb_config_desc_t* config = nullptr;
    if (usb_host_get_config_desc(client, handle, value, &config) != ESP_OK || config == nullptr) {
        ESP_LOGI(kTag, "config %u: not available", value);
        return;
    }
    ESP_LOGI(kTag, "config %u: %u interface(s), %u bytes", value, config->bNumInterfaces,
             config->wTotalLength);
    const std::uint8_t* const base = reinterpret_cast<const std::uint8_t*>(config);
    const int total = config->wTotalLength;
    for (int offset = 0; offset + 2 <= total;) {
        const std::uint8_t length = base[offset];
        const std::uint8_t type = base[offset + 1];
        if (length < 2U) {
            break;
        }
        if (type == USB_B_DESCRIPTOR_TYPE_INTERFACE && length >= USB_INTF_DESC_SIZE) {
            const auto* const interface = reinterpret_cast<const usb_intf_desc_t*>(base + offset);
            ESP_LOGI(kTag, "  cfg%u intf %u alt %u class %02x/%02x eps %u", value,
                     interface->bInterfaceNumber, interface->bAlternateSetting,
                     interface->bInterfaceClass, interface->bInterfaceSubClass,
                     interface->bNumEndpoints);
        } else if (type == USB_B_DESCRIPTOR_TYPE_ENDPOINT && length >= USB_EP_DESC_SIZE) {
            const auto* const endpoint = reinterpret_cast<const usb_ep_desc_t*>(base + offset);
            ESP_LOGI(kTag, "    cfg%u ep %02x type %u mps %u", value, endpoint->bEndpointAddress,
                     endpoint->bmAttributes & 0x03U,
                     static_cast<unsigned>(endpoint->wMaxPacketSize & 0x07FFU));
        }
        offset += length;
    }
}

// One claim attempt. Returns true when the interface is claimed and a transfer is
// in flight, false when the caller should try again (or give up, if the device
// left).
bool try_claim(usb_device_handle_t handle) {
    MidiEndpoint endpoint{};
    if (!find_midi_endpoint(handle, endpoint)) {
        return false;
    }
    static bool dumpedOtherConfigurations{};
    if (!dumpedOtherConfigurations) {
        dumpedOtherConfigurations = true;
        // The release number is what a vendor asks for first when a descriptor
        // turns out to be non-compliant, so it goes in the log next to the
        // layout.
        const usb_device_desc_t* descriptor = nullptr;
        if (usb_host_get_device_descriptor(handle, &descriptor) == ESP_OK &&
            descriptor != nullptr) {
            ESP_LOGI(kTag, "device %04x:%04x bcdUSB %04x bcdDevice %04x", descriptor->idVendor,
                     descriptor->idProduct, descriptor->bcdUSB, descriptor->bcdDevice);
        }
        dump_configuration(handle, 2U);
    }
    clamp_full_speed_packet_sizes(handle);
    if (endpoint.mps > kControllerRxPacketMax) {
        endpoint.mps = kControllerRxPacketMax;
    }
    const esp_err_t claim = usb_host_interface_claim(client, handle, endpoint.interfaceNumber,
                                                     endpoint.alternateSetting);
    if (claim == ESP_ERR_NO_MEM) {
        // The controller's DMA descriptors come out of internal SRAM, and the
        // audio client is still claiming its own interfaces at this point.
        return false;
    }
    if (claim != ESP_OK) {
        ESP_LOGW(kTag, "could not claim MIDI interface %u alt %u: %s", endpoint.interfaceNumber,
                 endpoint.alternateSetting, esp_err_to_name(claim));
        return false;
    }
    claimedInterface = endpoint.interfaceNumber;
    // Room for a burst, not one packet. The first transfer asked for 64 bytes --
    // one packet at the clamped MPS -- and the device answered with more than it
    // had been asked for, which the controller reports as an overflow and halts
    // the endpoint. 512 is a multiple of the 64-byte packet size, so it is a
    // legal length for this endpoint, and a short packet still completes it.
    const std::size_t transferBytes = endpoint.mps;
    if (usb_host_transfer_alloc(transferBytes, 0, &transfer) != ESP_OK || transfer == nullptr) {
        ESP_LOGW(kTag, "could not allocate MIDI transfer");
        usb_host_interface_release(client, handle, claimedInterface);
        claimedInterface = 0xFFU;
        return false;
    }
    transfer->device_handle = handle;
    transfer->bEndpointAddress = endpoint.address;
    transfer->callback = transfer_callback;
    transfer->context = nullptr;
    device = handle;
    pendingDevice = nullptr;
    ESP_LOGI(kTag, "MIDI interface %u claimed, endpoint 0x%02x mps=%u", endpoint.interfaceNumber,
             endpoint.address, endpoint.mps);
    submit_transfer();
    return true;
}

void client_event(const usb_host_client_event_msg_t* const event, void*) noexcept {
    if (event->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        if (deviceOpen) {
            return;
        }
        usb_device_handle_t handle = nullptr;
        if (usb_host_device_open(client, event->new_dev.address, &handle) != ESP_OK ||
            handle == nullptr) {
            return;
        }
        deviceOpen = true;
        pendingDevice = handle;
        // The claim is attempted from its own task: it needs to wait for internal
        // SRAM, and waiting here would stall this client's event loop.
        xQueueSend(setupQueue, &handle, 0);
        return;
    }
    if (event->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        const usb_device_handle_t gone = event->dev_gone.dev_hdl;
        if (gone == nullptr) {
            return;
        }
        if (gone == device) {
            // Stop the callback before anything is released: it checks device.
            device = nullptr;
            if (transfer != nullptr) {
                // A pending transfer completes with NO_DEVICE and frees itself.
                return;
            }
            release_device(gone);
            return;
        }
        if (gone == pendingDevice) {
            pendingDevice = nullptr;
            deviceOpen = false;
        }
    }
}

void setup_task(void*) noexcept {
    while (true) {
        usb_device_handle_t handle = nullptr;
        if (xQueueReceive(setupQueue, &handle, pdMS_TO_TICKS(5000)) != pdTRUE) {
            ESP_LOGI(kTag, "midi: messages=%u pending=%d claimed=%d",
                     messageCount.load(std::memory_order_relaxed),
                     transferPending.load(std::memory_order_relaxed) ? 1 : 0,
                     claimedInterface != 0xFFU ? 1 : 0);
            continue;
        }
        bool claimed = false;
        for (int attempt = 0; attempt < kClaimAttempts && !claimed; ++attempt) {
            if (pendingDevice != handle) {
                // Gone, or replaced by a replug.
                break;
            }
            claimed = try_claim(handle);
            if (!claimed) {
                if (attempt == 0) {
                    ESP_LOGW(kTag, "MIDI claim deferred; internal free=%u largest=%u",
                             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
                             static_cast<unsigned>(
                                 heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)));
                }
                vTaskDelay(pdMS_TO_TICKS(kClaimRetryDelayMs));
            }
        }
        if (!claimed && pendingDevice == handle) {
            ESP_LOGW(kTag, "MIDI interface not claimed after %d attempts; switches disabled",
                     kClaimAttempts);
            pendingDevice = nullptr;
            usb_host_device_close(client, handle);
            deviceOpen = false;
        }
    }
}

void midi_task(void*) noexcept {
    usb_host_client_config_t config{};
    config.is_synchronous = false;
    config.max_num_event_msg = 8;
    config.async.client_event_callback = client_event;
    config.async.callback_arg = nullptr;
    // The audio client installs the host library from its own task, and this one
    // is created right after it, so registration can arrive before the library
    // exists -- which fails with ESP_ERR_INVALID_STATE rather than waiting.
    for (int waited_ms = 0; !usb_audio_host_installed(); waited_ms += 20) {
        if (waited_ms >= 15000) {
            ESP_LOGE(kTag, "USB host library never came up; MIDI switches disabled");
            vTaskDelete(nullptr);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    const esp_err_t result = usb_host_client_register(&config, &client);
    if (result != ESP_OK) {
        ESP_LOGE(kTag, "could not register MIDI USB client: %s", esp_err_to_name(result));
        vTaskDelete(nullptr);
        return;
    }
    // A device that enumerated before this client registered never produces a
    // NEW_DEV event for it, and the XTONE is normally already attached when the
    // pedal boots. Adopt whatever is on the bus now, then live on events.
    vTaskDelay(pdMS_TO_TICKS(200));
    std::uint8_t addresses[8]{};
    int count = 0;
    if (usb_host_device_addr_list_fill(static_cast<int>(sizeof addresses), addresses, &count) ==
        ESP_OK) {
        for (int index = 0; index < count && index < static_cast<int>(sizeof addresses); ++index) {
            usb_device_handle_t handle = nullptr;
            if (usb_host_device_open(client, addresses[index], &handle) != ESP_OK ||
                handle == nullptr) {
                continue;
            }
            if (deviceOpen) {
                usb_host_device_close(client, handle);
                continue;
            }
            deviceOpen = true;
            pendingDevice = handle;
            xQueueSend(setupQueue, &handle, 0);
        }
    }
    while (true) {
        usb_host_client_handle_events(client, portMAX_DELAY);
    }
}

void worker_task(void*) noexcept {
    Action action{};
    while (true) {
        if (xQueueReceive(actions, &action, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        if (action.kind == Action::LoadPreset) {
            if (coyopedal_ui_control_load_preset(action.index)) {
                ESP_LOGI(kTag, "preset %u loaded from MIDI", static_cast<unsigned>(action.index));
            } else {
                ESP_LOGW(kTag, "preset %u load failed", static_cast<unsigned>(action.index));
            }
            continue;
        }
        const auto block = static_cast<coyopedal_fx_block_t>(action.index);
        const bool enable = !coyopedal_fx_enabled(block);
        if (coyopedal_ui_control_set_enabled(action.index, enable)) {
            ESP_LOGI(kTag, "block %u %s from MIDI", static_cast<unsigned>(action.index),
                     enable ? "on" : "off");
        } else {
            ESP_LOGW(kTag, "block %u toggle failed", static_cast<unsigned>(action.index));
        }
    }
}

} // namespace

void usb_midi_start() {
    if (actions == nullptr) {
        actions = xQueueCreate(8, sizeof(Action));
    }
    if (setupQueue == nullptr) {
        setupQueue = xQueueCreate(2, sizeof(usb_device_handle_t));
    }
    if (actions == nullptr || setupQueue == nullptr) {
        ESP_LOGE(kTag, "could not create the MIDI queues");
        return;
    }
    // The worker touches flash and NVS, so its stack must be internal: a PSRAM
    // stack cannot run while the flash cache is disabled.
    if (xTaskCreatePinnedToCore(worker_task, "usb_midi_work", kWorkerStackBytes, nullptr,
                                kWorkerPriority, nullptr, kMidiCore) != pdPASS) {
        ESP_LOGE(kTag, "could not create the MIDI worker");
        return;
    }
    if (xTaskCreatePinnedToCoreWithCaps(setup_task, "usb_midi_setup", kSetupTaskStackBytes, nullptr,
                                        kSetupTaskPriority, nullptr, kMidiCore,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(kTag, "could not create the MIDI setup task");
        return;
    }
    // The client only parses packets and enqueues; its stack can live in PSRAM.
    if (xTaskCreatePinnedToCoreWithCaps(midi_task, "usb_midi", kMidiTaskStackBytes, nullptr,
                                        kMidiTaskPriority, nullptr, kMidiCore,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGE(kTag, "could not create the MIDI client");
    }
}

const char* usb_midi_last_message() {
    return lastMessageText;
}

const char* usb_midi_mode_name() {
    return lastGroup.load();
}

std::uint32_t usb_midi_message_count() {
    return messageCount.load(std::memory_order_relaxed);
}
