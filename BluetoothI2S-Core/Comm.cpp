// Comm handler serial communications for debug and co-processor integration

#include <driver/uart.h>
#include <driver/gpio.h>
#include <string.h>
#include "I2C.h"
#include "Audio.h"
#include "Comm.h"

// Metadata configuration
#define METADATA_FIELDS 4
#define METADATA_LENGTH_MAX 128

// Debug communication
#define DBG_SERIAL  UART_NUM_0
#define DBG_TX      GPIO_NUM_1
#define DBG_RX      GPIO_NUM_3
#define DBG_BAUD    115200

// CO-Processor communication
#define COP_SERIAL  UART_NUM_2
#define COP_TX      GPIO_NUM_17
#define COP_RX      GPIO_NUM_16
#define COP_BAUD    1000000

// Misc
#define PRINTF_BUF_SIZE 128

// Proprietary "magic" δ-Law table
const uint16_t dlaw_table[] = {
    0x0000, 0x0001, 0x0002, 0x0003, 0x0004, 0x0005, 0x0006, 0x0007,
    0x0008, 0x0009, 0x000A, 0x000B, 0x000C, 0x000D, 0x000F, 0x0010,
    0x0011, 0x0013, 0x0015, 0x0017, 0x0019, 0x001C, 0x001E, 0x0021,
    0x0024, 0x0027, 0x002B, 0x002E, 0x0033, 0x0037, 0x003C, 0x0042,
    0x0048, 0x004F, 0x0056, 0x005E, 0x0067, 0x0070, 0x007A, 0x0085,
    0x0091, 0x009E, 0x00AD, 0x00BD, 0x00CE, 0x00E1, 0x00F6, 0x010C,
    0x0124, 0x013F, 0x015C, 0x017B, 0x019E, 0x01C3, 0x01EC, 0x0219,
    0x024A, 0x0280, 0x02BA, 0x02FA, 0x0340, 0x038B, 0x03DE, 0x0438,
    0x049B, 0x0506, 0x057C, 0x05FD, 0x068B, 0x0726, 0x07D0, 0x088A,
    0x0954, 0x0A2C, 0x0B10, 0x0BFE, 0x0CF3, 0x0DF2, 0x0EFF, 0x1026,
    0x116F, 0x12E2, 0x1480, 0x1641, 0x181C, 0x1A0C, 0x1C12, 0x1E32,
    0x2078, 0x22ED, 0x2590, 0x285B, 0x2B4C, 0x2E64, 0x31A9, 0x3521,
    0x38CA, 0x3C9F, 0x40A1, 0x44D2, 0x4931, 0x4DC2, 0x5286, 0x577B,
    0x5CA1, 0x61F8, 0x6783, 0x6D43, 0x733B, 0x7968, 0x7FC4, 0x864B,
    0x8CFB, 0x93D7, 0x9AE3, 0xA221, 0xA991, 0xB130, 0xB8F2, 0xC0CB,
    0xC8B0, 0xD099, 0xD884, 0xE06D, 0xE853, 0xF038, 0xF81C, 0xFFFF,
};

// Transmit management
SemaphoreHandle_t txPending;
char txMetadata[METADATA_FIELDS][METADATA_LENGTH_MAX];
SemaphoreHandle_t txMetadataAccess;

// Callbacks
static void (*cb_command)(Command &);

// Streaming audio sources
typedef enum {
  STREAM_NONE,
  STREAM_INPUT,
  STREAM_OUTPUT,
} stream_type;

// Streaming audio flag
static stream_type live_stream = STREAM_NONE;

// Message types
typedef enum {
  MSG_I2CX    = 0x2E,
  MSG_META    = 0x4D,
  MSG_AUDIO   = 0x41,
  MSG_COMMAND = 0x43,
} message_type;

typedef enum {
  I2CX_WS = 0x01,
  I2CX_RS = 0x02,
  I2CX_WW = 0x03,
  I2CX_RW = 0x04,
} i2cx_opcode;

// Export comm driver
Comm_Driver Comm;

// Find the δ-Law code to best approximate word
static uint8_t dlaw_search(uint16_t word) {
    uint8_t lo = 0, hi = 127, code = 0;
    while(lo <= hi) {
        uint8_t n = (lo + hi) >> 1;
        if(dlaw_table[n] > word) {
            hi = n - 1;
        } else {
            code = n;
            lo = n + 1;
						if(lo >= 128 || dlaw_table[lo] > word) break;
        }
    }
    return code;
}

// Variadic formatted print to serial port
static void uart_vprintf(uart_port_t uart_num, const char* format, va_list args) {
  char buf[PRINTF_BUF_SIZE];
  int len = vsnprintf(buf, sizeof(buf), format, args);
  if(len <= 0) return;
  if(len > PRINTF_BUF_SIZE - 1) len = PRINTF_BUF_SIZE - 1;
  uart_write_bytes(uart_num, buf, len);
}

/*
// Formatted print to serial port
static void uart_printf(uart_port_t uart_num, const char* format, ...) {
  va_list args;
  va_start(args, format);
  uart_vprintf(uart_num, format, args);
  va_end(args);
}
*/

// Formatted print to debug serial port
void Comm_Driver::printf(const char* format, ...) {
  va_list args;
  va_start(args, format);
  uart_vprintf(DBG_SERIAL, format, args);
  va_end(args);
}

int Comm_Driver::available() {
  size_t len = 0;
  uart_get_buffered_data_len(DBG_SERIAL, &len);
  return (int)len;
}

int Comm_Driver::read() {
  uint8_t byte = 0;
  int read_len = uart_read_bytes(DBG_SERIAL, &byte, 1, 0);
  if(read_len == 1) return byte;
  return -1;
}

// Send packet to CO-Processor
static void cop_send(uint8_t * packet, uint8_t packet_size) {
  uint8_t *scan = &packet[++packet_size], *next = scan;
  *packet = *scan = 0;
  while(scan > packet) {
    if(*--scan == 0) {
      *scan = (uint8_t)(next - scan);
      next = scan;
    }
  }
  uart_write_bytes(COP_SERIAL, packet, ++packet_size);
}

// Trims UTF-8 string to a maximum length in bytes
static void utf8_trim(char* str, size_t length) {
  size_t i = 0;
  while(i < length) {
    uint8_t c = (uint8_t)str[i];
    size_t codepoint = 0;
    if     ((c & 0x80) == 0)    codepoint = 1; // ASCII
    else if((c & 0xE0) == 0xC0) codepoint = 2; // 2-byte UTF-8
    else if((c & 0xF0) == 0xE0) codepoint = 3; // 3-byte UTF-8
    else if((c & 0xF8) == 0xF0) codepoint = 4; // 4-byte UTF-8
    else break;                               // Invalid byte
    if(i + codepoint > length) break;       // Break before last character
    i += codepoint;
  }
  str[i] = 0;
}

// Handle packet from CO-Processor
static void cop_handler(uint8_t * packet, uint8_t packet_size) {
  uint8_t resp[256];
  if(packet[0] == MSG_I2CX) {
    uint8_t *in = packet + 1;
    uint8_t *end = packet + packet_size;
    uint8_t *out = resp + 2;

    resp[1] = MSG_I2CX;
    I2C.acquire();
    while(in < end) {
      uint8_t op = *in++;
      uint8_t addr, reg8, len;
      uint16_t reg16;

      if(op == I2CX_WS) {
        if((end - in) < 3) break;
        addr = *in++;
        reg8 = *in++;
        len = *in++;
        if((end - in) < len) break;
        I2C.write(addr, reg8, in, len);
        if((resp + sizeof(resp) - out) < 4) break;
        *out++ = op;
        *out++ = addr;
        *out++ = reg8;
        *out++ = len;
        in += len;
      } else if(op == I2CX_RS) {
        if((end - in) < 3) break;
        addr = *in++;
        reg8 = *in++;
        len = *in++;
        if((resp + sizeof(resp) - out) < (uint16_t)(4 + len)) break;
        *out++ = op;
        *out++ = addr;
        *out++ = reg8;
        *out++ = len;
        I2C.read(addr, reg8, out, len);
        out += len;
      } else if(op == I2CX_WW) {
        if((end - in) < 4) break;
        addr = *in++;
        reg16 = ((uint16_t)in[0] << 8) | in[1];
        in += 2;
        len = *in++;
        if((end - in) < len) break;
        I2C.write(addr, reg16, in, len, true);
        if((resp + sizeof(resp) - out) < 5) break;
        *out++ = op;
        *out++ = addr;
        *out++ = reg16 >> 8;
        *out++ = reg16;
        *out++ = len;
        in += len;
      } else if(op == I2CX_RW) {
        if((end - in) < 4) break;
        addr = *in++;
        reg16 = ((uint16_t)in[0] << 8) | in[1];
        in += 2;
        len = *in++;
        if((resp + sizeof(resp) - out) < (uint16_t)(5 + len)) break;
        *out++ = op;
        *out++ = addr;
        *out++ = reg16 >> 8;
        *out++ = reg16;
        *out++ = len;
        I2C.read(addr, reg16, out, len, true);
        out += len;
      } else {
        break;
      }
    }
    I2C.release();
    if(out > resp + 2) {
      cop_send(resp, (uint8_t)(out - resp - 1));
    }
  } else if(packet[0] == MSG_AUDIO) {
    if(packet_size < 2) return;
    if(packet[1] == 'I') {
      live_stream = STREAM_INPUT;
    } else if(packet[1] == 'O') {
      live_stream = STREAM_OUTPUT;
    } else {
      live_stream = STREAM_NONE;
    }
  } else if(packet[0] == MSG_COMMAND) {
    if(cb_command) {
      Command command(&packet[1], packet_size - 1);
      cb_command(command);
    }
  }
}

// Receive data from CO-Processor
static void rx_task(void *dummy) {
  uint8_t packet[256];
  uint8_t packet_size = 0;
  uint8_t cobs_count = 0;
  uint8_t *cobs_ptr = packet;
  uint8_t q;
  while(true) {
    uart_read_bytes(COP_SERIAL, &q, 1, portMAX_DELAY);
    //uart_write_bytes(COP_SERIAL, &q, 1);
    if(q == 0) {
      // Encountered sync character
      if(cobs_count == 0 && packet_size >= 2 && packet_size < 255) {
        // COBS encapsulation seems good, process
        cop_handler(packet + 1, packet_size - 1);
      }
      // Reset
      cobs_count = 0;
      cobs_ptr = packet;
      packet_size = 0;
    } else {
      // If it's not a sync character, it's data
      if(packet_size == 254) {
        // Too much data was received = communications error
        cobs_count = 255;  // Set error state
      } else {
        if(cobs_count == 0) {
          // Time to read a COBS-marker, treat as 0x00
          cobs_count = q - 1;
          q = 0x00;
        } else cobs_count--;
        // Store octet
        *cobs_ptr = q;
        // Progress pointer
        cobs_ptr++;
        // Track packet length
        packet_size++;
      }
    }
  }
}

// Transmit data to CO-Processor
static void tx_task(void *dummy) {
  uint8_t resp[METADATA_LENGTH_MAX + 4];
  while(1) {
    xSemaphoreTake(txPending, portMAX_DELAY);

    // Send metadata
    resp[1] = MSG_META;
    for(uint8_t n = 0; n < METADATA_FIELDS; n++) {
      resp[2] = n;
      xSemaphoreTake(txMetadataAccess, portMAX_DELAY);
      if(txMetadata[n][0]) {
        uint8_t length = strlen(txMetadata[n]);
        memcpy(&resp[3], txMetadata[n], length);
        txMetadata[n][0] = 0;
        xSemaphoreGive(txMetadataAccess);
        cop_send(resp, length + 2);
      } else {
        xSemaphoreGive(txMetadataAccess);
      }
    }

  }
}

// AVRC metadata from Bluetooth source
static void comm_meta(uint8_t id, const char * data, size_t length) {  
  // * 0x01 Title of the media (track title)
  // * 0x02 Artist name
  //   0x03 Album name
  //   0x04 Track number
  //   0x05 Total number of tracks in album
  //   0x06 Genre
  // * 0x07 Playing time in milliseconds
  uint8_t index;    
       if(id == 0x01) index = 1;
  else if(id == 0x02) index = 2;
  else if(id == 0x07) index = 3;
  else return;
  if(length > METADATA_LENGTH_MAX - 1) length = METADATA_LENGTH_MAX - 1;
  xSemaphoreTake(txMetadataAccess, portMAX_DELAY);
  memcpy(txMetadata[index], data, length);
  utf8_trim(txMetadata[index], length);
  xSemaphoreGive(txMetadataAccess);
  xSemaphoreGive(txPending);
}

// AVRC connection to Bluetooth source changed
static void comm_connection(bool connected, const char * name, size_t length) {
  if(length > METADATA_LENGTH_MAX - 1) length = METADATA_LENGTH_MAX - 1;
  xSemaphoreTake(txMetadataAccess, portMAX_DELAY);  
  if(connected) {
    if(length == 0) {
      strcpy(txMetadata[0], "Bluetooth");
      length = strlen(txMetadata[0]);
    } else {
      memcpy(txMetadata[0], name, length);
    }
    utf8_trim(txMetadata[0], length);
  } else {
    txMetadata[0][0] = 255;
    txMetadata[0][1] = 0;
  }
  xSemaphoreGive(txMetadataAccess);
  xSemaphoreGive(txPending);
}

// Audio samples
static void comm_audio(int16_t *samples, bool input) {
  if((live_stream == STREAM_INPUT && input) || (live_stream == STREAM_OUTPUT && !input)) {
    uint8_t resp[4 + AUDIO_CHUNK_SIZE / 4]; // AUDIO_CHUNK / 2(monomix) / 2(δ-Law)
    resp[1] = MSG_AUDIO;
    size_t nsamples = AUDIO_CHUNK_SIZE / 4; // AUDIO_CHUNK / 2(int16) / 2(stereo);
    int8_t *dst = (int8_t*)&resp[2];
    // Start with first sample known
    int16_t pcm_tx = *(int16_t*)dst = ((int32_t)samples[0] + samples[1]) >> 1;
    samples += 2;
    dst += 2;
    while(--nsamples) {
      int16_t pcm_rx = ((int32_t)samples[0] + samples[1]) >> 1;
      samples += 2;
      int32_t word = (int32_t)pcm_rx - pcm_tx;
      int8_t code = dlaw_search(word ^ (word >> 31)) | ((word >> 24) & 0x80);
      word = dlaw_table[code & 0x7F];
      pcm_tx += word ^ (int16_t)(code >> 7);
      *dst++ = code;
    }
    // Send live audio
    cop_send(resp, sizeof(resp) - 2);
  }
}

void (*Comm_Driver::getMetaHandler())(uint8_t, const char *, size_t) {
  return comm_meta;
}

void (*Comm_Driver::getConnectionHandler())(bool, const char *, size_t) {
  return comm_connection;
}

void (*Comm_Driver::getAudioHandler())(int16_t *, bool) {
  return comm_audio;
}

void Comm_Driver::setCommandHandler(void (*cb)(Command &)) {
  cb_command = cb;
}

void Comm_Driver::init() {
  uart_config_t uart_config;
  
  uart_config = (uart_config_t){
    .baud_rate = DBG_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(DBG_SERIAL, &uart_config);
  uart_set_pin(DBG_SERIAL, DBG_TX, DBG_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(DBG_SERIAL, 256, 256, 0, NULL, 0);
  
  uart_config = (uart_config_t){
    .baud_rate = COP_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
  };
  uart_param_config(COP_SERIAL, &uart_config);
  uart_set_pin(COP_SERIAL, COP_TX, COP_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(COP_SERIAL, 256, 256, 0, NULL, 0);

  // Semaphores
  txPending = xSemaphoreCreateBinary();
  txMetadataAccess = xSemaphoreCreateBinary();
  xSemaphoreGive(txMetadataAccess);

  // Co-processor communicator
  xTaskCreate(rx_task, "COP RX", 2048, NULL, 1, NULL);  
  xTaskCreate(tx_task, "COP TX", 2048, NULL, 1, NULL);  
}
