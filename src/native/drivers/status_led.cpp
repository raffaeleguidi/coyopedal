// The status LED.
//
// A headless board has no screen, so the preset number is shown the one way
// that needs no hardware: the addressable LED on the devkit blinks it out. One
// blink for preset 1, two for preset 2, and so on.
//
// The LED is driven from a low-priority task, never from the task that recalled
// the preset: a blink is a second of delays, and the recall path must return to
// the audio promptly. The task's stack is in PSRAM -- it only formats a frame
// and sleeps.

#include "status_led.h"

#include <cstdint>

#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace {

constexpr const char* kTag = "status_led";

// The addressable LED the DevKitC-1 carries. Revision 1.0 wires it to GPIO48 and
// revision 1.1 moved it to GPIO38; this board is a bare N16R8 module, so the pin
// lives here rather than in the target and moving it is this one line.
constexpr gpio_num_t kLedGpio = GPIO_NUM_48;

// 0.1 us per tick. A WS2812 bit is 1.2 us: 0.3 us high for a zero, 0.9 us for a
// one, the rest low.
constexpr std::uint32_t kResolutionHz = 10000000U;
constexpr std::uint16_t kShortHighTicks = 3U;
constexpr std::uint16_t kLongHighTicks = 9U;
constexpr std::uint16_t kShortLowTicks = 9U;
constexpr std::uint16_t kLongLowTicks = 3U;

constexpr std::uint8_t kFull = 0xFFU;
constexpr std::uint32_t kOnMs = 220U;
constexpr std::uint32_t kOffMs = 220U;
constexpr std::uint32_t kGapMs = 400U;
constexpr std::uint32_t kTaskStackBytes = 3072U;
constexpr UBaseType_t kTaskPriority = 5U;

rmt_channel_handle_t channel{};
rmt_encoder_handle_t encoder{};
QueueHandle_t requests{};

bool write_pixel(const std::uint8_t red, const std::uint8_t green, const std::uint8_t blue) {
    if (channel == nullptr || encoder == nullptr) {
        return false;
    }
    // A WS2812 takes the channels in green, red, blue order.
    const std::uint8_t grb[3] = {green, red, blue};
    rmt_transmit_config_t config{};
    config.loop_count = 0;
    if (rmt_transmit(channel, encoder, grb, sizeof grb, &config) != ESP_OK) {
        return false;
    }
    return rmt_tx_wait_all_done(channel, 100) == ESP_OK;
}

void blink_task(void*) {
    unsigned count = 0U;
    while (true) {
        if (xQueueReceive(requests, &count, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        for (unsigned index = 0; index < count; ++index) {
            (void)write_pixel(kFull, kFull, kFull);
            vTaskDelay(pdMS_TO_TICKS(kOnMs));
            (void)write_pixel(0U, 0U, 0U);
            vTaskDelay(pdMS_TO_TICKS(kOffMs));
        }
        // The gap separates two preset changes that land close together, and the
        // low line between blinks is what latches the frame.
        vTaskDelay(pdMS_TO_TICKS(kGapMs));
    }
}

} // namespace

void status_led_start() {
    if (requests != nullptr) {
        return;
    }
    rmt_tx_channel_config_t channel_config{};
    channel_config.gpio_num = kLedGpio;
    channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    channel_config.resolution_hz = kResolutionHz;
    channel_config.mem_block_symbols = 64;
    channel_config.trans_queue_depth = 2;
    if (rmt_new_tx_channel(&channel_config, &channel) != ESP_OK || channel == nullptr) {
        channel = nullptr;
        ESP_LOGW(kTag, "status LED unavailable: no RMT channel");
        return;
    }
    rmt_bytes_encoder_config_t encoder_config{};
    encoder_config.bit0.level0 = 1;
    encoder_config.bit0.duration0 = kShortHighTicks;
    encoder_config.bit0.level1 = 0;
    encoder_config.bit0.duration1 = kShortLowTicks;
    encoder_config.bit1.level0 = 1;
    encoder_config.bit1.duration0 = kLongHighTicks;
    encoder_config.bit1.level1 = 0;
    encoder_config.bit1.duration1 = kLongLowTicks;
    encoder_config.flags.msb_first = 1;
    if (rmt_new_bytes_encoder(&encoder_config, &encoder) != ESP_OK || encoder == nullptr) {
        ESP_LOGW(kTag, "status LED unavailable: no encoder");
        return;
    }
    if (rmt_enable(channel) != ESP_OK) {
        ESP_LOGW(kTag, "status LED unavailable: channel will not enable");
        return;
    }
    requests = xQueueCreate(1, sizeof(unsigned));
    if (requests == nullptr) {
        ESP_LOGW(kTag, "status LED unavailable: no queue");
        return;
    }
    if (xTaskCreatePinnedToCoreWithCaps(blink_task, "status_led", kTaskStackBytes, nullptr,
                                        kTaskPriority, nullptr, 0,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) != pdPASS) {
        ESP_LOGW(kTag, "status LED unavailable: no task");
    }
}

void status_led_blink(const unsigned count) {
    if (requests == nullptr || count == 0U) {
        return;
    }
    (void)xQueueOverwrite(requests, &count);
}
