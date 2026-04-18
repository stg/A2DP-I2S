#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "Audio.h"
#include "Biquad.h"

#define SGTL5000_GAIN_LO  -90.0
#define SGTL5000_GAIN_HI    0.0
#define SGTL5000_BQ_COUNT   7
#define SGTL5000_BQ_Q      18

typedef enum {
  SGTL5000_HP_DAC = 0,
  SGTL5000_HP_LINEIN = 1,
} sgtl5000_headphone;

typedef enum {
  SGTL5000_IN_LINEIN = 0,
  SGTL5000_IN_MIC = 1,
} sgtl5000_input;

typedef enum {
  SGTL5000_RAMP_EXP,
  SGTL5000_RAMP_LINEAR,
  SGTL5000_RAMP_DISABLE,
} sgtl5000_dac_ramp;

typedef enum {
  SGTL5000_HPF_ENABLE,
  SGTL5000_HPF_DISABLE,
  SGTL5000_HPF_FREEZE,
} sgtl5000_hpf;

typedef enum {
  SGTL5000_DAP_DISABLE,
  SGTL5000_DAP_PRE,
  SGTL5000_DAP_POST,
} sgtl5000_dap_mode;

typedef enum {
  SGTL5000_EQ_NONE = 0,
  SGTL5000_EQ_PEQ = 1,
  SGTL5000_EQ_TONE = 2,
  SGTL5000_EQ_GEQ = 3,
} sgtl5000_eq;

typedef enum {
  SGTL5000_CUTOFF_80HZ = 0,
  SGTL5000_CUTOFF_100HZ = 1,
  SGTL5000_CUTOFF_125HZ = 2,
  SGTL5000_CUTOFF_150HZ = 3,
  SGTL5000_CUTOFF_175HZ = 4,
  SGTL5000_CUTOFF_200HZ = 5,
  SGTL5000_CUTOFF_225HZ = 6,
} sgtl5000_cutoff;

typedef enum {
  SGTL5000_AVC_MAX_GAIN_0DB = 0,
  SGTL5000_AVC_MAX_GAIN_6DB = 1,
  SGTL5000_AVC_MAX_GAIN_12DB = 2,
} sgtl5000_avc_max_gain;

typedef enum {
  SGTL5000_AVC_SMOOTH_0MS   = 0,
  SGTL5000_AVC_SMOOTH_25MS  = 1,
  SGTL5000_AVC_SMOOTH_50MS  = 2,
  SGTL5000_AVC_SMOOTH_100MS = 3,
} sgtl5000_avc_smooth;

class SGTL5000_Driver : public Audio_Codec {
  uint8_t _address;    
  uint16_t _id; // Should read 0xA0xx (0xA011 at time of writing)
  uint16_t write(uint16_t reg, uint16_t data);
  uint16_t read(uint16_t reg);
  void delay(int time);
  void writeDapBlock(uint16_t start_reg, const uint16_t *data, uint8_t count);
  
public:
  SGTL5000_Driver(uint8_t address) : _address(address) {}

  uint8_t  address()      const override { return _address;          }
  uint32_t id()           const override { return _id;               }
  float    volumeMin()    const override { return SGTL5000_GAIN_LO;  }
  float    volumeMax()    const override { return SGTL5000_GAIN_HI;  }
  uint8_t  biquadCount()  const override { return SGTL5000_BQ_COUNT; }
	uint8_t  biquadFormat() const override { return SGTL5000_BQ_Q;     }

	void init(audio_samplerate samplerate) override;
	void hpVolumeRaw(uint8_t left, uint8_t right);
	void hpVolume(float left_norm, float right_norm);
	void hpVolume(float vol_norm);
	void lineOutLevel(uint8_t level); // range: 13..31
	void lineOutLevel(uint8_t left, uint8_t right); // range: each 13..31
	void lineInGain(uint8_t level); // range: 0..15
	void lineInGain(uint8_t left, uint8_t right); // range: each 0..15
	void micSetupRaw(uint16_t mic_ctrl, uint16_t ana_adc_ctrl);
	void micSetup(unsigned int gain_db, float bias_v = 0);
	void analogPathSetup(sgtl5000_input input, sgtl5000_headphone headphone, bool headphone_mute, bool lineout_mute);
	void volumeRaw(uint8_t l_vol, uint8_t r_vol);
	void volume(float l_gain_db, float r_gain_db = NAN) override;
	void dacPathSetup(sgtl5000_dac_ramp ramp, sgtl5000_hpf hpf, audio_channel mute = CH_NONE);
	void dapSetup(sgtl5000_dap_mode mode);
	void eqModeSetup(sgtl5000_eq eq, uint8_t peqCount = 7);
	void geqBandSetRaw(uint8_t band, uint8_t gain);
	void geqBandSet(uint8_t band, float gain_norm);
	void geqSetup(float bass_norm, float mid_bass_norm, float midrange_norm, float mid_treble_norm, float treble_norm);
	void geqSetup(float bass_norm, float treble_norm);
	void biquadSet(audio_channel ch, uint8_t stage, biquad_filter &filter) override;
	void biquadSet(audio_channel ch, uint8_t stage, biquad_filter_q &filter) override;
	void avcSetupRaw(uint16_t threshold, uint16_t attack, uint16_t decay, uint16_t avc_ctrl);
	void avcSetup(bool enabled, sgtl5000_avc_max_gain max_gain, sgtl5000_avc_smooth smooth, bool hard_limit, float threshold_db, float attack_dbps, float decay_dbps);
	void bassEnhanceSetupRaw(uint16_t bass_enhance, uint16_t bass_enhance_ctrl);
	void bassEnhanceSetup(bool enabled, float lr_mix_norm, float bass_gain_norm, bool hpf_bypass = false, sgtl5000_cutoff cutoff = SGTL5000_CUTOFF_80HZ);
	void surroundSetupRaw(uint16_t surround);
	void surroundSetup(bool enabled, uint8_t width, uint8_t select = 0);
	void setMasterMode(uint32_t freq_mclk_in);
};
