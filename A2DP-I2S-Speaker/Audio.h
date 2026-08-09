#ifndef __AUDIO_H__
#define __AUDIO_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Resources.h"
#include "Biquad.h"

// Size of each chunk of audio processing (in bytes)
#define AUDIO_CHUNK_SIZE_N 7 // 128
#define AUDIO_CHUNK_SIZE (1 << AUDIO_CHUNK_SIZE_N)

typedef enum {
  SR_32K  = 32000,
  SR_44K1 = 44100,
  SR_48K  = 48000,
  SR_96K  = 96000,
} audio_samplerate;

typedef enum {
  SRC_ANALOG,
  SRC_BT,
} audio_source;

typedef enum {
  EVT_READY,
  EVT_BT_CONNECT,
  EVT_BT_DISCONNECT,
  EVT_BT_START,
  EVT_BT_STOP,
  EVT_BT_STABLE,
  EVT_SYNTH_DONE,
  EVT_FADE_DONE,
  EVT_CUSTOM,
} audio_event;

typedef enum {
  CMD_NONE,
  CMD_PLAY,
  CMD_PAUSE,
  CMD_STOP,
  CMD_NEXT,
  CMD_PREV,
  CMD_FWD,
  CMD_REW,
  CMD_VOLINC,
  CMD_VOLDEC,
} audio_command;

typedef enum {
  CH_NONE = 0,
  CH_LEFT = 1,
  CH_RIGHT = 2,
  CH_BOTH = 3
} audio_channel;

class Audio_Codec {
public:
  virtual ~Audio_Codec() {}
  virtual void init(audio_samplerate samplerate) = 0;
  virtual void volume(float l_gain_db, float r_gain_db = NAN) = 0;
  virtual uint8_t address() const = 0;
  virtual uint32_t id() const = 0;
  virtual float volumeMin() const { return 0; }
  virtual float volumeMax() const { return 0; }
  virtual uint8_t biquadCount() const { return 0; }
  virtual uint8_t biquadFormat() const { return 0; }
  virtual void biquadSet(audio_channel ch, uint8_t stage, biquad_filter &filter) = 0;
  virtual void biquadSet(audio_channel ch, uint8_t stage, biquad_filter_q &filter) = 0;
};

typedef struct {
  int32_t b32v;
  uint8_t b32q;
  int32_t b32;
  int32_t f32;
  uint32_t t;
} sweep_data;

class Audio_Driver {
public:
  Audio_Driver() {}
  void         init(const char *, audio_samplerate, bool preemption = false, bool mclk = true);
  void         begin();
  uint16_t     samplerateSource();
  uint16_t     samplerateSink();
  float        samplerateRatio();
  float        bufferLevel();
  audio_source currentSource();
  void         setMetaHandler(void (*)(uint8_t, const char *, size_t));
  void         setConnectionHandler(void (*)(bool, const char *, size_t));
  void         setAudioHandler(void (*)(int16_t *, bool));
  void         setEventHandler(void (*)(audio_event));
  void         startSynthMP3(res_id mp3);
  void         startSynthSweep(sweep_data seed);
  audio_event  getEvent();
  void         postEvent(int);
  void         startFade(float);
  void         setSource(audio_source);
  audio_source getSource();
  void         sendCommand(audio_command);
  void         sendCommand(audio_command command, bool pressed);
  void         disconnectBluetooth();
};

extern Audio_Driver Audio;

#endif
