#include <stdint.h>
#include <stdbool.h>
#include "Audio.h"
#include "Biquad.h"

#define TAS5828M_GAIN_LO  -104.0f
#define TAS5828M_GAIN_HI   +24.0f
#define TAS5828M_AGAIN_LO  -15.5f
#define TAS5828M_AGAIN_HI    0.0f
#define TAS5828M_BQ_COUNT   16
#define TAS5828M_BQ_Q       27

typedef enum {
  TAS5828M_BD = 0,
  TAS5828M_1SPW = 1,
  TAS5828M_HYBRID = 2,
} tas5828m_modulation;

typedef enum {
  TAS5828M_MONO = 1,
  TAS5828M_STEREO = 0,
} tas5828m_output;

typedef enum {
  TAS5828M_DEFAULT = 0,
  TAS5828M_INVERTED = 1,
  TAS5828M_OPENDRAIN = 2,
  TAS5828M_INVERTED_OPENDRAIN = 3
} tas5828m_pinmode;

typedef enum {
  TAS5828M_1FS = 0,
  TAS5828M_2FS = 1,
  TAS5828M_4FS = 2,
  TAS5828M_INSTANT = 3,
} tas5828m_ramp_speed;

typedef enum {
  TAS5828M_4DB = 0,
  TAS5828M_2DB = 1,
  TAS5828M_1DB = 2,
  TAS5828M_0DB5 = 3,
} tas5828m_ramp_step;

typedef enum {
  TAS5828M_11MS5 = 0,
  TAS5828M_53MS = 1,
  TAS5828M_106MS5 = 2,
  TAS5828M_266MS5 = 3,
  TAS5828M_535MS = 4,
  TAS5828M_1S065 = 5,
  TAS5828M_2S665 = 6,
  TAS5828M_5S33 = 7,
} tas5828m_automute_time;

typedef enum {
  TAS5828M_DEEP_SLEEP = 0,
  TAS5828M_SLEEP = 1,
  TAS5828M_HIZ = 2,
  TAS5828M_PLAY = 3
} tas5828m_state;

typedef union {
  uint32_t raw;
  struct {
    uint32_t r_overcurrent_fault : 1; // CH2_OC_I
    uint32_t l_overcurrent_fault : 1; // CH1_OC_I
    uint32_t right_dc_fault : 1; // CH2_DC_1
    uint32_t left_dc_fault : 1; // CH1_DC_1
    uint32_t reserved_0 : 4;

    uint32_t pvdd_undervoltage_fault : 1; // PVDD_UV_I
    uint32_t pvdd_overvoltage_fault : 1; // PVDD_OV_I
    uint32_t clock_fault : 1; // CLK_FAULT_I
    uint32_t reserved_1 : 2;
    uint32_t eeprom_load_error : 1; // LOAD_EEPROM_ERROR
    uint32_t biquad_write_error : 1; // BQ_WR_ERROR
    uint32_t otp_crc_error : 1; // OTP_CRC_ERROR

    uint32_t overtemp_shutdown : 1; // OTSD_I
    uint32_t l_cbc_overcurrent_fault : 1; // CBC_FAULT_CH1_I
    uint32_t r_cbc_overcurrent_fault : 1; // CBC_FAULT_CH2_I
    uint32_t reserved_2 : 5;

    uint32_t otw1_flag : 1; // OTW1_FLAG
    uint32_t otw2_flag : 1; // OTW2_FLAG
    uint32_t otw3_flag : 1; // OTW3_FLAG
    uint32_t otw4_flag : 1; // OTW4_FLAG
    uint32_t ch2_cbcw_flag : 1; // CH2CBCW_FLAG
    uint32_t ch1_cbcw_flag : 1; // CH1CBCW_FLAG
    uint32_t reserved_3 : 2;
  };
} tas5828m_status_t;

typedef union {
  uint32_t raw;
  struct {
    uint32_t fs_code : 4; // FS
    uint32_t sclk_ratio_high : 2; // SCLK_RATIO_HIGH
    uint32_t reserved_0 : 2;

    uint32_t sclk_ratio_low : 8; // BCLK(SCLK)_RATIO_LOW

    uint32_t fs_valid : 1; // DET_STATUS_FS_VALID
    uint32_t sclk_valid : 1; // DET_STATUS_SCLK_VALID
    uint32_t sclk_missing : 1; // DET_STATUS_SCLK_MISSING
    uint32_t pll_locked : 1; // DET_STATUS_PLL_LOCKED
    uint32_t pll_overrate : 1; // DET_STATUS_PLL_OVERRATE
    uint32_t sclk_range_error : 1; // DET_STATUS_SCLK_RANGE_ERROR
    
    uint32_t reserved_1 : 10;
  };
} tas5828m_clockmon_t;

class TAS5828M_Driver : public Audio_Codec {
  uint8_t _address;
  uint8_t _id; // Should read 0x98 despite datasheet saying 0x95 (confirmed by support)
  
  void write(uint8_t reg, uint8_t data);
  void write(uint8_t reg, uint8_t * data, uint8_t length);
  uint8_t read(uint8_t reg);
  void read(uint8_t reg, uint8_t * data, uint8_t length);
  void delay(int time);
  void setBookPage(uint8_t book, uint8_t page);
  void writeDspBlock(uint8_t book, uint8_t page, uint8_t addr, const uint32_t *data, uint8_t words);

public:
  TAS5828M_Driver(uint8_t address) : _address(address) {}

  uint8_t  address()      const override { return _address;          }
  uint32_t id()           const override { return _id;               }
  float    volumeMin()    const override { return TAS5828M_GAIN_LO;  }
  float    volumeMax()    const override { return TAS5828M_GAIN_HI;  }
  uint8_t  biquadCount()  const override { return TAS5828M_BQ_COUNT; }
  uint8_t  biquadFormat() const override { return TAS5828M_BQ_Q;     }

  void init(audio_samplerate samplerate) override;
  void init(audio_samplerate samplerate, tas5828m_output output, tas5828m_modulation modulation = TAS5828M_1SPW, tas5828m_pinmode pwm_pinmode = TAS5828M_INVERTED);

  // Real-world equalizer example (dampen treble, kill resonance peak):
  //   biquad_filter f;
  //   biquadSet(CH_BOTH, 0, f = biquadMake(FILTER_HIGHSHELF, fs, 4000.0f, 0.5f, -1.5f));
  //   biquadSet(CH_BOTH, 1, f = biquadMake(FILTER_PEAK, fs, 1280.0f, 1.5f, -2.8f));
  
  // Engineering template:
  //   biquad_filter f;
  //   biquadSet(channel_select, stage_select, f = biquadMake(filter_type, sr, fc_norm, q, gain_db));

  void biquadSet(audio_channel ch, uint8_t stage, biquad_filter &filter) override;
  void biquadSet(audio_channel ch, uint8_t stage, biquad_filter_q &filter) override;

  void volume(float l_gain_db, float r_gain_db = NAN) override;

  void volumeRaw(int32_t l_vol, int32_t r_vol);
  void analogGainRaw(uint8_t raw);
  void analogGain(float gain_db);
  void gainRaw(uint8_t raw);
  void gain(float gain_db);
  void volumeRamp(tas5828m_ramp_speed down_speed, tas5828m_ramp_step down_step, tas5828m_ramp_speed up_speed = TAS5828M_1FS, tas5828m_ramp_step up_step = TAS5828M_0DB5);
  void volumeEmergencyRamp(tas5828m_ramp_speed down_speed, tas5828m_ramp_step down_step);
  void autoMute(bool left_enable, bool right_enable, bool both_channels_together, tas5828m_automute_time left_time, tas5828m_automute_time right_time);
  void pathSelect(audio_channel left, audio_channel right); //  CH_BOTH is invalid 
  tas5828m_state state();
  void setState(tas5828m_state state);
  tas5828m_clockmon_t clockMon();
  audio_channel autoMuteState();
  tas5828m_status_t status();
  void clearFaults();
  uint8_t pvddRaw();
  float pvdd();

  // Real-world input crossbar example (swap L/R, invert R phase):
  //   inputSetup(1.0, 0.0, 0.0, -1.0);

  // Input mixer 2x2 (Q9.23):
  // outL = inL*l_from_l + inR*l_from_r, outR = inL*r_from_l + inR*r_from_r
  //
  // Engineering template:
  //   inputSetup(
  //     powf(10.0f, left_from_left_db   / 20.0f),
  //     powf(10.0f, left_from_right_db  / 20.0f),
  //     powf(10.0f, right_from_left_db  / 20.0f),
  //     powf(10.0f, right_from_right_db / 20.0f)
  //   );

  void inputSetupRaw(uint32_t l_from_l = 0x00800000, uint32_t l_from_r = 0x00000000, uint32_t r_from_l = 0x00000000, uint32_t r_from_r = 0x00800000);
  void inputSetup(float l_from_l = 1.0f, float l_from_r = 0.0f, float r_from_l = 0.0f, float r_from_r = 1.0f);

  // Real-world crossbar example (center removal):
  //   crossbarSetup(0.5, -0.5, -0.5, 0.5);

  // Output crossbar 2x2 + 2x2 (Q9.23): amp path and I2S path
  // x_outL = inL*x_l_from_l + inR*x_l_from_r, x_outR = inL*x_r_from_l + inR*x_r_from_r (x=dig,ana)
  //
  // Engineering template:
  //  crossbarSetup(
  //     powf(10.0f, amp_left_from_left_db      / 20.0f),
  //     powf(10.0f, amp_left_from_right_db     / 20.0f),
  //     powf(10.0f, amp_right_from_left_db     / 20.0f),
  //     powf(10.0f, amp_right_from_right_db    / 20.0f),
  //     powf(10.0f, i2s_left_from_left_db      / 20.0f),
  //     powf(10.0f, i2s_left_from_right_db     / 20.0f),
  //     powf(10.0f, i2s_right_from_left_db     / 20.0f),
  //     powf(10.0f, i2s_right_from_right_db    / 20.0f)
  //   );

  void crossbarSetupRaw(uint32_t amp_l_from_l = 0x00800000, uint32_t amp_l_from_r = 0x00000000, uint32_t amp_r_from_l = 0x00000000, uint32_t amp_r_from_r = 0x00800000, uint32_t i2s_l_from_l = 0x00800000, uint32_t i2s_l_from_r = 0x00000000, uint32_t i2s_r_from_l = 0x00000000, uint32_t i2s_r_from_r = 0x00800000);
  void crossbarSetup(float amp_l_from_l = 1.0f, float amp_l_from_r = 0.0f, float amp_r_from_l = 0.0f, float amp_r_from_r = 1.0f, float i2s_l_from_l = 1.0f, float i2s_l_from_r = 0.0f, float i2s_r_from_l = 0.0f, float i2s_r_from_r = 1.0f);

  // Real-world clipper example (clip at 50% full-scale):
  //   clipperSetup(2.0f, 0.5f, 0.5f);

  // Clipper: presscale into clipping region, postscale to recover
  // Uses undisclosed soft clipping envelope
  //
  // Engineering template:
  //   clipperSetup(
  //     powf(10.0f, -clip_level_db / 20.0f), // pre-gain (boost into clipper)
  //     powf(10.0f,  clip_level_db / 20.0f), // post-gain (restore after clipping)
  //     powf(10.0f,  clip_level_db / 20.0f)  // post-gain (restore after clipping)
  //   );

  void clipperSetupRaw(uint32_t prescale = 0x00800000, uint32_t l_postscale = 0x40000000, uint32_t r_postscale = 0x40000000);
  void clipperSetup(float prescale = 1.0f, float l_postscale = NAN, float r_postscale = NAN);

  // Real-world dynamic parametric equalizer example (bass booster +6db@-30dB, flat@-10dB):
  //   biquad_filter f;
  //   dpeqBiquadSet(0, f = biquadMake(FILTER_NONE, fs, 0));
  //   dpeqBiquadSet(1, f = biquadMake(FILTER_LOWSHELF, fs, 90.0f, 0.707f, +6.0f));
  //   dpeqBiquadSet(2, f = biquadMake(FILTER_NONE, fs, 0));
  //   dpeqSetup(1.0f - expf(-1000.0f / (fs * 120.0)),
  //             1.0f / (powf(10.0f, -10 / 20.0f) - powf(10.0f, -30  / 20.0f)),
  //             powf(10.0f, -30 / 20.0f));

  // DPEQ biquads - stage index 0..2: sense, low, high
  // Defines filters for level detection (sense) and dynamic EQ bands (low/high).
  // Sense filter drives envelope detection, low/high filters apply level-dependent EQ.
  //
  // Engineering template: see biquadSet()

  void dpeqBiquadSet(uint8_t stage, biquad_filter &filter);
  void dpeqBiquadSet(uint8_t stage, biquad_filter_q &filter);

  // DPEQ coefficients - alpha, threshold gain, threshold offset.
  //
  // Engineering template:
  //   dpeqSetup(
  //     1.0f - expf(-1000.0f / (sr * smooth_ms)),
  //     1.0f / (powf(10.0f, threshold_high_db / 20.0f) - powf(10.0f, threshold_low_db / 20.0f)),
  //     powf(10.0f, threshold_low_db / 20.0f)
  //   );

  void dpeqSetupRaw(uint32_t alpha = 0x02DEAD00, uint32_t gain = 0x74013901, uint32_t offset = 0x0020C49B);
  void dpeqSetup(float alpha = 0.0224205f, float gain = 0.9062873f, float offset = 0.001f);

  // 3-band dynamic range compressor real-world example (ugly, agressive, loudness-war compressor):
  //   float xover_lo_hz         =  250.0f;
  //   float xover_hi_hz         = 3500.0f;
  //   float smooth_ms           =    5.0f; // Quite fast
  //   float attack_ms           =   10.0f;
  //   float decay_ms            =   50.0f;
  //   float low_slope_db_per_db =   2.20f;
  //   float pt1_in_db           =  -68.0f;
  //   float pt1_out_db          =  -36.0f; // +32 dB lift
  //   float pt2_in_db           =  -30.0f;
  //   float pt2_out_db          =  -6.0f;  // +24 dB lift
  //   float high_slope_db_per_db = pt2_out_db / pt2_in_db; // 0 dB @ 0 dB
  //   biquad_filter f;
  //   drc3BiquadSet(0, f = biquadMake(FILTER_LOWPASS,  sr, xover_lo_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(1, f = biquadMake(FILTER_LOWPASS,  sr, xover_lo_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(2, f = biquadMake(FILTER_HIGHPASS, sr, xover_lo_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(3, f = biquadMake(FILTER_HIGHPASS, sr, xover_lo_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(4, f = biquadMake(FILTER_LOWPASS,  sr, xover_hi_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(5, f = biquadMake(FILTER_LOWPASS,  sr, xover_hi_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(6, f = biquadMake(FILTER_HIGHPASS, sr, xover_hi_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   drc3BiquadSet(7, f = biquadMake(FILTER_HIGHPASS, sr, xover_hi_hz, 1.0f / sqrtf(2.0f), 0.0f));
  //   for(unsigned n = 0; n < 3; n++) {
  //       drc3TimeSetup(n,
  //           expf(-1000.0f / (sr * smooth_ms)),
  //           expf(-1000.0f / (sr * attack_ms)),
  //           expf(-1000.0f / (sr *  decay_ms))
  //       );
  //       drc3CurveSetup(n,
  //           low_slope_db_per_db - 1.0f,
  //           ((pt2_out_db - pt1_out_db) / (pt2_in_db - pt1_in_db)) - 1.0f,
  //           high_slope_db_per_db - 1.0f,
  //           pt1_in_db / (20.0f * log10f(2.0f)) - 4.0f,
  //           pt2_in_db / (20.0f * log10f(2.0f)) - 4.0f,
  //           (pt1_out_db - pt1_in_db) / (20.0f * log10f(2.0f)),
  //           (pt2_out_db - pt2_in_db) / (20.0f * log10f(2.0f))
  //       );
  //   }
  //   drc3MixerSetup(
  //     powf(10.0f, -12.0f / 20.0f),
  //     powf(10.0f, -12.0f / 20.0f),
  //     powf(10.0f, -12.0f / 20.0f)
  //   );
  
  // 3-band DRC biquad - stage index 0..7: low[0..1], mid[2..5], high[6..7]
  // Defines crossover filters for bands, typically LP for low-band, LP+HP for mid-band, HP for high-band
  // Engineering template: see biquadSet()

  void drc3BiquadSet(uint8_t stage, biquad_filter &filter);
  void drc3BiquadSet(uint8_t stage, biquad_filter_q &filter);

  // 3DRC timing coefficients - band index low[0], mid[1], high[2]
  //
  // Engineering template:
  //   drc3TimeSetup(b,
  //     expf(-1000.0f / (sr * smooth_ms)),
  //     expf(-1000.0f / (sr * attack_ms)), 
  //     expf(-1000.0f / (sr *  decay_ms)));

  void drc3TimeSetupRaw(uint8_t band, uint32_t alpha = 0x7FFFFFFF, uint32_t attack = 0x7FFFFFFF, uint32_t decay = 0x7FFFFFFF);
  void drc3TimeSetup(uint8_t band, float alpha = 1.0f, float attack = 1.0f, float decay = 1.0f);
  
  // 3DRC transfer curve - k0/k1/k2: ratios, t1/t2 thresholds, off1/off2 offsets
  // Internally different than other blocks, looks like piecewise affine transform on a level variable
  // Implies linear/level mapping elsewhere, likely LUT-based (TODO: where are LUTs?)
  //
  // Engineering template:
  //   drc3CurveSetup(
  //     0,
  //     low_slope_db_per_db - 1.0f,
  //     ((pt2_out_db - pt1_out_db) / (pt2_in_db - pt1_in_db)) - 1.0f,
  //     high_slope_db_per_db - 1.0f,
  //     pt1_in_db / (20.0f * log10f(2.0f)) - 4.0f,
  //     pt2_in_db / (20.0f * log10f(2.0f)) - 4.0f,
  //     (pt1_out_db - pt1_in_db) / (20.0f * log10f(2.0f)),
  //     (pt2_out_db - pt2_in_db) / (20.0f * log10f(2.0f))
  //   );

  void drc3CurveSetupRaw(uint8_t band, uint32_t k0 = 0x00000000, uint32_t k1 = 0x00000000, uint32_t k2 = 0x00000000, uint32_t t1 = 0xE7000000, uint32_t t2 = 0xFE800000, uint32_t off1 = 0x00000000, uint32_t off2 = 0x00000000);
  void drc3CurveSetup(uint8_t band, float k0 = 0.0f, float k1 = 0.0f, float k2 = 0.0f, float t1 = -50.0f, float t2 = -3.0f, float off1 = 0.0f, float off2 = 0.0f);
  
  // 3DRC mixer gains (post-processing recombination)
  //
  // Engineering template:
  //   drc3MixerSetup(
  //     powf(10.0f, low_band_gain_db  / 20.0f), // Low-band gain
  //     powf(10.0f, mid_band_gain_db  / 20.0f), // Mid-band gain
  //     powf(10.0f, high_band_gain_db / 20.0f)  // High-band gain
  //   );

  void drc3MixerSetupRaw(uint32_t drc1 = 0x00800000, uint32_t drc2 = 0x00000000, uint32_t drc3 = 0x00000000);
  void drc3MixerSetup(float drc1 = 1.0f, float drc2 = 0.0f, float drc3 = 0.0f);

  // Automatic Gain Limiter real-world example:
  // TODO
  
  // AGL envelope-based limiter coefficients
  // - alpha: smoothing, attack/decay: rate-controlled gain change,
  // - omega: smoothing, temp/volt: gain reduction in response to temp/pvdd
  // Note: attack/decay/other units and effective operation not fully explored
  //
  // Engineering template:
  //   aglSetup(
  //     true,
  //     powf(10.0f, threshold_db / 20.0f),
  //     1.0f / (sr * attack_ms),
  //     1.0f / (sr *  decay_ms),
  //     1.0f - expf(-1000.0f / (sr * alpha_ms)),
  //     omega_norm,                              // Typically fixed at appx. 0.96
  //     powf(10.0f, temp_scale_db / 20.0f),      // 0dB = disabled
  //     powf(10.0f, volt_scale_db / 20.0f),      // 0dB = disabled
  //     powf(10.0f, volt_temp_scale_db / 20.0f)  // 0dB = neutral
  //   );

  void aglSetupRaw(uint32_t alpha = 0x051EB852, uint32_t attack = 0x000369D0, uint32_t enable = 0x40000000, uint32_t omega = 0x7AE147AE, uint32_t decay = 0x00005762, uint32_t threshold = 0x40000000, uint32_t temp_scale = 0x00800000, uint32_t volt_scale = 0x010D489D, uint32_t volt_temp_scale = 0x00080000);
  void aglSetup(bool enable = true, float threshold = 0.5f, float attack = 0.0001f, float decay = 0.00001f, float alpha = 0.04f, float omega = 0.96f, float temp_scale = 1.0f, float volt_scale = 2.1f, float volt_temp_scale = 0.0625f);

  // Hybrid/Class-H PWM generator real-world example (4ms lookahead@48kHz, 100% a little early, 28.5Vrms@100%):
  //   hybridSetup(192, 0, 48, 0.99f, 0.95f, 0.05f, threshold = 0.8f);
  //   analogGain(-1.5); // TAS5830 analog setup for 28.5Vrms Class-H supply max (-1.5dB=27.9V/30.4Vmax) (TAS5828M Vmax=28.5)
  
  // Hybrid (class-h) PWM output setup - delay/window/hold: detector, peak_*: response shaping, threshold: 100% command level
  //
  // Engineering template:
  //   hybridSetup(
  //     delay_samples,                           // audio peak lookahead delay
  //     window_samples,                          // effective operation not confirmed, typically 0
  //     hold_samples,                            // peak hold time
  //     peak_offset_norm,                        // extra rail headroom above detected peak
  //     1.0f - expf(-1000.0f / (sr * decay_ms)), // peak envelope decay coefficient
  //     peak_smooth_norm,                        // command smoothing / damping coefficient
  //     powf(10.0f, threshold_db / 20.0f)        // level that maps to 100% rail command
  //   );

  void hybridSetupRaw(uint32_t l_delay_samples = 240, uint32_t r_delay_samples = 240, uint32_t window_samples = 0, uint32_t hold_samples = 480, uint32_t peak_offset = 0x7FDF3B64, uint32_t peak_decay = 0x7999999A, uint32_t peak_smooth = 0x053947A6, uint32_t threshold = 0x06666666);
  void hybridSetup(uint32_t delay_samples = 240, uint32_t window_samples = 0, uint32_t hold_samples = 480, float peak_offset = 0.999f, float peak_decay = 0.95f, float peak_smooth = 0.04f, float threshold = 0.8f);
};
