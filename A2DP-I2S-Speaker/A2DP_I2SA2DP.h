#pragma once

#if defined(A2DP_I2S_CUSTOM_CORE)

#include <stddef.h>
#include <stdint.h>
#include <esp_gap_bt_api.h>
#include <esp_avrc_api.h>
#include <a2dp_i2s_a2dp.h>

struct i2s_channel_obj_t;

#ifndef A2DP_I2S_TEST_DROP_PACKET_PERIOD
#define A2DP_I2S_TEST_DROP_PACKET_PERIOD 0
#endif
#ifndef A2DP_I2S_TEST_DROP_PACKET_BURST
#define A2DP_I2S_TEST_DROP_PACKET_BURST 1
#endif

class BluetoothA2DPOutput {
public:
  virtual ~BluetoothA2DPOutput() = default;
  virtual bool begin() = 0;
  virtual void end() = 0;
  virtual void set_output_active(bool active) = 0;
  virtual void set_sample_rate(int rate) = 0;
  virtual size_t write(const uint8_t *data, size_t length,
                       uint64_t packet_arrival_us,
                       uint32_t source_sample_index) = 0;
};

class A2DP_I2SA2DPSink {
public:
  explicit A2DP_I2SA2DPSink(BluetoothA2DPOutput &output) : output_(output) {}

  void begin_i2s(uint32_t sample_rate, bool use_mclk,
                 int8_t bclk_pin, int8_t ws_pin,
                 int8_t dout_pin, int8_t din_pin, int8_t mclk_pin,
                 i2s_channel_obj_t *&rx_channel,
                 i2s_channel_obj_t *&tx_channel);
  void start(const char *name);
  bool is_connected() const;
  const char *get_peer_name() const { return peer_name_; }
  void disconnect();

  void set_discoverability(esp_bt_discovery_mode_t mode);
  void set_avrc_metadata_attribute_mask(uint8_t mask) { metadata_mask_ = mask; }
  void set_avrc_metadata_callback(void (*callback)(uint8_t, const uint8_t *, size_t)) { metadata_callback_ = callback; }
  void set_avrc_connection_state_callback(void (*callback)(bool)) { connection_callback_ = callback; }
  void set_max_write_delay_ms(int) {}
  void enable_preemption(bool enable) { preemption_ = enable; }

  void play() { passthrough(ESP_AVRC_PT_CMD_PLAY); }
  void pause() { passthrough(ESP_AVRC_PT_CMD_PAUSE); }
  void stop() { passthrough(ESP_AVRC_PT_CMD_STOP); }
  void next() { passthrough(ESP_AVRC_PT_CMD_FORWARD); }
  void previous() { passthrough(ESP_AVRC_PT_CMD_BACKWARD); }
  void fast_forward() { passthrough(ESP_AVRC_PT_CMD_FAST_FORWARD); }
  void rewind() { passthrough(ESP_AVRC_PT_CMD_REWIND); }
  void volume_up() { passthrough(ESP_AVRC_PT_CMD_VOL_UP); }
  void volume_down() { passthrough(ESP_AVRC_PT_CMD_VOL_DOWN); }
  void send_passthrough(esp_avrc_pt_cmd_t operation, bool pressed) {
    rb_avrc_passthrough(static_cast<uint8_t>(operation), pressed);
  }

private:
  static void pcm_callback(const int16_t *samples, size_t bytes,
                           uint64_t packet_arrival_us,
                           uint32_t source_sample_index, void *context);
  static void event_callback(rb_a2dp_event_t event, uint32_t value, void *context);
  static void metadata_callback(uint8_t attribute, const uint8_t *text,
                                size_t length, void *context);
  void passthrough(uint8_t operation);

  BluetoothA2DPOutput &output_;
  const char *peer_name_ = "unknown";
  void (*metadata_callback_)(uint8_t, const uint8_t *, size_t) = nullptr;
  void (*connection_callback_)(bool) = nullptr;
  uint8_t metadata_mask_ = 0;
  bool preemption_ = false;
#if A2DP_I2S_TEST_DROP_PACKET_PERIOD > 0
  uint32_t test_packet_count_ = 0;
  uint64_t test_packet_arrival_us_ = 0;
  bool test_packet_valid_ = false;
  bool test_drop_current_packet_ = false;
#endif
};

#endif
