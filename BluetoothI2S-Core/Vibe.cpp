// Vibe is a real-time DSP programming layer connecting AVRCP media session data to Audio_Codec

#include <Arduino.h>
#include <string.h>
#include "Biquad.h"
#include "Comm.h"
#include "Vibe.h"

Vibe_Driver Vibe;

typedef void (*metadata_cb)(uint8_t id, const char *data, size_t length);

static metadata_cb volatile cb_metadata;
static Audio_Codec *biquad_target;

#define LIVE_FILTER_COUNT 12
static biquad_filter_q live_filters[LIVE_FILTER_COUNT];

static uint8_t b64_lut_char(char c) {
  if (c >= 'A' && c <= 'Z') return (uint8_t)(c - 'A');
  if (c >= 'a' && c <= 'z') return (uint8_t)(26 + (c - 'a'));
  if (c >= '0' && c <= '9') return (uint8_t)(52 + (c - '0'));
  if (c == '+') return 62;
  return 63;
}

static bool is_b64_char(char c) {
  return (c >= 'A' && c <= 'Z')
      || (c >= 'a' && c <= 'z')
      || (c >= '0' && c <= '9')
      || c == '+'
      || c == '/';
}

static void decode_b64_56(const char *in, uint8_t *out) {
  size_t out_idx = 0;
  for (size_t i = 0; i < 56; i += 4) {
    uint32_t v = ((uint32_t)b64_lut_char(in[i + 0]) << 18)
               | ((uint32_t)b64_lut_char(in[i + 1]) << 12)
               | ((uint32_t)b64_lut_char(in[i + 2]) << 6)
               |  (uint32_t)b64_lut_char(in[i + 3]);
    out[out_idx++] = (uint8_t)(v >> 16);
    if (out_idx < 42) out[out_idx++] = (uint8_t)(v >> 8);
    if (out_idx < 42) out[out_idx++] = (uint8_t)v;
  }
}

static uint16_t crc12_3gpp(const uint8_t *bytes, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)(bytes[i] << 4);
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x800) crc = (uint16_t)(((crc << 1) ^ 0x80F) & 0x0FFF);
      else crc = (uint16_t)((crc << 1) & 0x0FFF);
    }
  }
  return (uint16_t)(crc & 0x0FFF);
}

static int32_t read_i32_be(const uint8_t *p) {
  return (int32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3]);
}

static void decode_one_filter(const uint8_t *payload, size_t offset) {
  const uint8_t index = payload[offset];
  if (index >= LIVE_FILTER_COUNT) return;
  biquad_filter_q next;
  next.b[0] = read_i32_be(&payload[offset + 1]);
  next.b[1] = read_i32_be(&payload[offset + 5]);
  next.b[2] = read_i32_be(&payload[offset + 9]);
  next.a[0] = read_i32_be(&payload[offset + 13]);
  next.a[1] = read_i32_be(&payload[offset + 17]);
  if (memcmp(&live_filters[index], &next, sizeof(biquad_filter_q)) != 0) {
    live_filters[index] = next;
    if (biquad_target) {
      biquad_target->biquadSet(CH_BOTH, index, live_filters[index]);
    }
    //Comm.printf("[Vibe] Biquad %u updated\n", index);
    //Comm.printf("b0 = %+11ld\n", (long)live_filters[index].b[0]);
    //Comm.printf("b1 = %+11ld    a1 = %+11ld\n", (long)live_filters[index].b[1], (long)live_filters[index].a[0]);
    //Comm.printf("b2 = %+11ld    a2 = %+11ld\n", (long)live_filters[index].b[2], (long)live_filters[index].a[1]);
  }
}

// Returns true if decoded and checksum matches.
// Input must be exactly 58 Base64 chars (56 payload + 2 CRC chars).
static bool live_filter_decode(const char *b64, size_t len) {
  if (!b64 || len != 58) return false;

  uint8_t payload[42];
  decode_b64_56(b64, payload);

  const char c0 = b64[56];
  const char c1 = b64[57];
  uint16_t crc_in = (uint16_t)(((uint16_t)b64_lut_char(c0) << 6) | (uint16_t)b64_lut_char(c1));
  uint16_t crc_calc = crc12_3gpp(payload, sizeof(payload));
  if (crc_in != crc_calc) return false;

  decode_one_filter(payload, 0);
  decode_one_filter(payload, 21);
  return true;
}

// Intercepts AVRCP metadata (id 0x01) to extract live filter updates.
static void vibe_meta_intercept(uint8_t id, const char *data, size_t length) {
  //Comm.printf("[Vibe] Key: %.*s\n", length, data);
  if (id == 0x01 && data && length == 59 && data[0] == 'F') {
    bool valid = true;
    for (size_t i = 1; i < length; i++) {
      if (!is_b64_char(data[i])) {
        valid = false;
        break;
      }
    }
    if (valid) {
      if (live_filter_decode(&data[1], length - 1)) {
        return;
      }
    }
  }
  if (cb_metadata) cb_metadata(id, data, length);
}

void (*Vibe_Driver::getMetaHandler())(uint8_t, const char *, size_t) {
  return vibe_meta_intercept;
}

void Vibe_Driver::setMetaHandler(void (*cb)(uint8_t, const char *, size_t)) {
  cb_metadata = cb;
}

void Vibe_Driver::setBiquadTarget(Audio_Codec &target) {
  biquad_target = &target;
}

void Vibe_Driver::clearBiquadTarget() {
  biquad_target = NULL;
}
