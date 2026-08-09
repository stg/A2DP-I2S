#include "A2DP_I2SA2DP.h"

#if defined(A2DP_I2S_CUSTOM_CORE)

#include <esp_bt.h>
#include <esp_bt_device.h>
#include <esp_err.h>
#include <esp_gap_bt_api.h>
#include <esp_bt_main.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <a2dp_i2s_a2dp.h>

namespace {
constexpr uint16_t kMediaMtu = 600;
constexpr uint32_t kI2sDmaFrames = 128;
constexpr uint32_t kI2sDmaDescriptors = 4;
}

void A2DP_I2SA2DPSink::begin_i2s(uint32_t sample_rate, bool use_mclk,
                                int8_t bclk_pin, int8_t ws_pin,
                                int8_t dout_pin, int8_t din_pin,
                                int8_t mclk_pin,
                                i2s_channel_obj_t *&rx_channel,
                                i2s_channel_obj_t *&tx_channel) {
  i2s_chan_config_t channel_config =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  channel_config.dma_frame_num = kI2sDmaFrames;
  channel_config.dma_desc_num = kI2sDmaDescriptors;
  channel_config.auto_clear = true;
  ESP_ERROR_CHECK(i2s_new_channel(
      &channel_config, &tx_channel, &rx_channel));

  i2s_std_config_t standard_config = {
    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg = {
      .mclk = use_mclk ? static_cast<gpio_num_t>(mclk_pin)
                       : I2S_GPIO_UNUSED,
      .bclk = static_cast<gpio_num_t>(bclk_pin),
      .ws = static_cast<gpio_num_t>(ws_pin),
      .dout = static_cast<gpio_num_t>(dout_pin),
      .din = static_cast<gpio_num_t>(din_pin),
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  standard_config.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_channel, &standard_config));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_channel, &standard_config));
  //if(use_mclk) gpio_set_drive_capability(static_cast<gpio_num_t>(mclk_pin), GPIO_DRIVE_CAP_2); // Changing this may vastly improve signal integrity
  ESP_ERROR_CHECK(i2s_channel_enable(tx_channel));
  ESP_ERROR_CHECK(i2s_channel_enable(rx_channel));
}

void A2DP_I2SA2DPSink::pcm_callback(const int16_t *samples, size_t bytes,
                                     uint64_t packet_arrival_us,
                                     uint32_t source_sample_index,
                                     void *context) {
  auto *self = static_cast<A2DP_I2SA2DPSink *>(context);
#if A2DP_I2S_TEST_DROP_PACKET_PERIOD > 0
  if (!self->test_packet_valid_ ||
      self->test_packet_arrival_us_ != packet_arrival_us) {
    self->test_packet_valid_ = true;
    self->test_packet_arrival_us_ = packet_arrival_us;
    const uint32_t period = A2DP_I2S_TEST_DROP_PACKET_PERIOD;
    const uint32_t burst = A2DP_I2S_TEST_DROP_PACKET_BURST < period
        ? A2DP_I2S_TEST_DROP_PACKET_BURST : period;
    const uint32_t position = self->test_packet_count_++ % period;
    self->test_drop_current_packet_ = position >= period - burst;
  }
  if (self->test_drop_current_packet_) return;
#endif
  self->output_.write(reinterpret_cast<const uint8_t *>(samples), bytes,
                      packet_arrival_us, source_sample_index);
}

void A2DP_I2SA2DPSink::event_callback(rb_a2dp_event_t event, uint32_t value, void *context) {
  auto *self = static_cast<A2DP_I2SA2DPSink *>(context);
  if (event == RB_A2DP_EVENT_CODEC_CONFIGURED) {
    self->output_.set_sample_rate(static_cast<int>(value));
  } else if (event == RB_A2DP_EVENT_STREAM_STARTED) {
    self->output_.set_output_active(true);
  } else if (event == RB_A2DP_EVENT_STREAM_SUSPENDED ||
             event == RB_A2DP_EVENT_DISCONNECTED) {
    self->output_.set_output_active(false);
  } else if (event == RB_A2DP_EVENT_AVRCP_CONNECTED) {
    if (self->connection_callback_) self->connection_callback_(true);
  } else if (event == RB_A2DP_EVENT_AVRCP_DISCONNECTED) {
    if (self->connection_callback_) self->connection_callback_(false);
  }
}

void A2DP_I2SA2DPSink::metadata_callback(uint8_t attribute, const uint8_t *text,
                                          size_t length, void *context) {
  auto *self = static_cast<A2DP_I2SA2DPSink *>(context);
  if (!self->metadata_callback_) return;
  self->metadata_callback_(attribute, text, length);
}

void A2DP_I2SA2DPSink::passthrough(uint8_t operation) {
  if (rb_avrc_passthrough(operation, true) == ESP_OK) {
    rb_avrc_passthrough(operation, false);
  }
}

void A2DP_I2SA2DPSink::start(const char *name) {
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
    esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    esp_bt_controller_config_t controller = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&controller));
  }
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
  }
  rb_bluedroid_set_audio_only(true);
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
    ESP_ERROR_CHECK(esp_bluedroid_init());
  }
  if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_INITIALIZED) {
    ESP_ERROR_CHECK(esp_bluedroid_enable());
  }

  esp_bt_dev_set_device_name(name);
  esp_bt_cod_t cod = {};
  cod.service = ESP_BT_COD_SRVC_RENDERING | ESP_BT_COD_SRVC_AUDIO;
  cod.major = ESP_BT_COD_MAJOR_DEV_AV;
  cod.minor = 0x05; // Audio/Video minor value; encoded on-air as 0x14.
  ESP_ERROR_CHECK(esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_ALL));
  output_.begin();
  rb_a2dp_set_media_mtu(kMediaMtu);
  rb_a2dp_config_t config = {
    .device_name = name,
    .sample_rate_mask = RB_A2DP_RATE_32000 | RB_A2DP_RATE_44100 | RB_A2DP_RATE_48000,
    .preferred_sample_rate = 48000,
    .pcm_callback = pcm_callback,
    .event_callback = event_callback,
    .metadata_attribute_mask = metadata_mask_,
    .preemption = preemption_,
    .metadata_callback = metadata_callback,
    .callback_context = this,
  };
  ESP_ERROR_CHECK(rb_a2dp_start(&config));
  set_discoverability(ESP_BT_GENERAL_DISCOVERABLE);
  ESP_ERROR_CHECK(esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9));
}

bool A2DP_I2SA2DPSink::is_connected() const {
  return rb_a2dp_connected();
}

void A2DP_I2SA2DPSink::disconnect() {
  rb_a2dp_disconnect();
}

void A2DP_I2SA2DPSink::set_discoverability(esp_bt_discovery_mode_t mode) {
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, mode);
}

#endif
