// OTA is an Over-The-Air firmware update utiliy that can be launched by a special key over AVRCP media session data

#include <Arduino.h>
#include "Comm.h"
#include "Audio.h"
#include "OTA.h"

#ifdef OTA_KEY

#include <BluetoothSerial.h>
#include <Update.h>

OTA_Driver OTA;

static BluetoothSerial SerialBT;
static void (* volatile cb_metadata)(uint8_t, const char *, size_t);

// OTA key
RTC_NOINIT_ATTR static uint8_t persistent_ota_key[sizeof(OTA_KEY) - 1];
static const char ota_key[sizeof(OTA_KEY)] = OTA_KEY;

static inline uint16_t ota_crc16(uint16_t init, const uint8_t *p, size_t n) {
  uint16_t crc = init;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc >>= 1;
    }
  }
  return crc;
}

static void ota_perform() {
  uint8_t buf[256], state = 0;
  uint32_t bitbuf = 1, len = 0, next_idx = 0, remaining = 0;
  while(SerialBT.hasClient()) {
    if(!SerialBT.available()) {
      delay(1);
    } else {
      uint8_t q =  SerialBT.read();
      if(q == '\n' || q == '\r') {
        if(len <= sizeof(buf) && len > 5) {
          uint16_t hdr_crc = ((uint16_t)buf[1] << 8) | buf[0];
          uint32_t hdr_idx = ((uint32_t)buf[4] << 16) | ((uint32_t)buf[3] << 8) | buf[2];
          uint16_t hdr_len = buf[5];
          if(
            (hdr_len == len - 6) &&
            (hdr_idx == next_idx) &&
            (hdr_crc == ota_crc16(0xC33D, &buf[2], len - 2)) &&
            (hdr_idx >= 4 || (hdr_idx == 0 && hdr_len == 4))
          ) {
            // accept
            next_idx += hdr_len;
            len = 0;
            if(hdr_idx == 0) {
              remaining = (uint32_t)buf[6] | ((uint32_t)buf[7] << 8) | ((uint32_t)buf[8] << 16) | ((uint32_t)buf[9] << 24);
              if(!Update.begin(remaining, U_FLASH)) {
                SerialBT.printf("-%s\n", Update.errorString());
                break;
              } else {
                state = 1;
                Comm.printf("[OTA] Begin\n");
                SerialBT.println("+begin");
              }            
            } else {
              if(hdr_len > remaining) {
                SerialBT.println("-file size exceeded");
                break;
              } else {
                size_t upd_len = 0;
                if(hdr_len) upd_len = Update.write(&buf[6], hdr_len);
                if(upd_len != (size_t)hdr_len) {
                  SerialBT.printf("-%s\n", Update.errorString());
                  break;
                } else {
                  remaining -= hdr_len;
                  if(remaining == 0) {
                    if(state != 2 && !Update.end(true)) { // note lazy evaluation
                      SerialBT.printf("-%s\n", Update.errorString());
                      break;
                    }
                    if(state != 2) Comm.printf("[OTA] End\n");
                    SerialBT.println("+end");
                    state = 2;
                  }
                }
              }
            }
          }
        }
        if(len) {
          SerialBT.printf("r%06X\n", next_idx); // request retransmit from next_idx
          len = 0;
        }
        bitbuf = 1;
      } else {
        do {
          if(q >= 'A' && q <= 'Z') q -= 'A';
          else if(q >= 'a' && q <= 'z') q -= 'a' - 26;
          else if(q >= '0' && q <= '9') q -= '0' - 52;
          else if(q == '+') q = 62;
          else if(q == '/') q = 63;
          else break;
          bitbuf = (bitbuf << 6) | q;
          uint8_t bits = 31 - __builtin_clz(bitbuf);
          if(bits >= 8) {
            bits -= 8;
            if(len < sizeof(buf)) buf[len] = (uint8_t)(bitbuf >> bits);
            if(len <= sizeof(buf)) len++;
            bitbuf = (bitbuf & ((1u << bits) - 1u)) | (1u << bits);
          }
        } while(0);
      }
    }
  }

  if(state == 1) Update.abort();
  SerialBT.disconnect();
  delay(1000);
  if(state == 2) {
    ESP.restart();
  }
}

static void ota_begin(const char * ota_name) {
  Comm.printf("[OTA] Waiting for firmware\n");
  SerialBT.begin(ota_name);
  uint8_t state = 0;
  uint32_t total_size = 0;
  for(uint32_t ota_count = 0; ota_count < 18000; ota_count++) {
    delay(10);
    if(SerialBT.hasClient()) {
      Comm.printf("[OTA] Connected\n");
      ota_perform();
      Comm.printf("[OTA] Disconnected\n");
      ota_count = 0;
    }
  }
  Comm.printf("[OTA] Timeout\n");
  SerialBT.end();
  ESP.restart();
}


// Intercept OTA key, persist it and reboot when found
static void ota_meta_intercept(uint8_t id, const char * data, size_t length) {
  //Comm.printf("[OTA] Key: %.*s\n", length, data);
  if(id == 0x01 && length == (sizeof(OTA_KEY) - 1) && !memcmp(ota_key, data, (sizeof(OTA_KEY) - 1))) {
    memcpy(persistent_ota_key, ota_key, (sizeof(OTA_KEY) - 1));
    Audio.disconnectBluetooth();
    delay(1000);
    ESP.restart();
  } else {
    if(cb_metadata) cb_metadata(id, data, length);
  }
}

// Begin OTA when persistant key is present
void OTA_Driver::init(const char * ota_name) {
  bool match = !memcmp(persistent_ota_key, ota_key, (sizeof(OTA_KEY) - 1));
  memset((void *)persistent_ota_key, 0, (sizeof(OTA_KEY) - 1));
  if(match) ota_begin(ota_name);
}

void (*OTA_Driver::getMetaHandler())(uint8_t, const char *, size_t) {
  return ota_meta_intercept;
}

void OTA_Driver::setMetaHandler(void (*cb)(uint8_t, const char *, size_t)) {
  cb_metadata = cb;
}

#endif
