// TAS5828 is an Audio_Codec driver for TAS5828M (primary), TAS5830 (tested), and TAS5837 (maybe, probably)

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <arpa/inet.h>
#include <math.h>
#include <bit>
#include "I2C.h"
#include "TAS5828M.h"

/*
This is by no means a full implementation of all "process flows", but should be sufficient for basic++ use.
Based on *many* months of testing and experimentation, but is not fully proven for all blocks.
Many implementation details for this chip are simply not publicly disclosed.

DSP processing sequence (default process flow 1):

   InputMixer => Equalizer => Volume => DPEQ => DRC => PostEQ1 => AGL => PostEQ2 => Clipper => Crossbar/ClassH

   See TAS5828M.h for examples and engineering templates for setting up the blocks.

   Well tested:
   - Equalizer: 12xBQ (configure with biquadSet 0..11)
   - PostEQ1: 2xBQ (configure with biquadSet 12..13)
   - PostEQ2: 2xBQ (configure with biquadSet 14..15)
   - ClassH (Hybrid-mode): runs parallel to Crossbar (configure with hybridSetup)
   - DPEQ: level-dependent crossfader between two filters (configure with dpeqSetup, dpeqBiquadSet 0..2)
   - InputMixer: 2x2 matrix (configure with inputSetup)
   - Crossbar: amp+i2s 2x2 matrices (configure with crossbarSetup)
   - Clipper: prescale/postscale path (configure with clipperSetup)

   Tested, seems to work, but is not fully confirmed:
   - DRC3: timing/curve/mixer (configure with drc3TimeSetup/drc3CurveSetup/drc3MixerSetup) and split BQs (configure with drc3BiquadSet)

   Not tested:
   - AGL: supply/thermal adaptive gain block (configure with aglSetup)
*/

// Abstraction
void    TAS5828M_Driver::write(uint8_t reg, uint8_t data) { I2C.write(_address, reg, data); }
void    TAS5828M_Driver::write(uint8_t reg, uint8_t * data, uint8_t length) { I2C.write(_address, reg, data, length); }
uint8_t TAS5828M_Driver::read(uint8_t reg) { return I2C.read(_address, reg); }
void    TAS5828M_Driver::read(uint8_t reg, uint8_t * data, uint8_t length) { I2C.read(_address, reg, data, length); }
void    TAS5828M_Driver::delay(int time) { vTaskDelay(time / portTICK_PERIOD_MS); }

// Big endian conversion for 32-bit
static uint32_t big32(uint32_t u32) {
  uint32_t x = 0x01020304;
  if(*reinterpret_cast<uint8_t*>(&x) == 0x04) {
    u32 = ((u32 & 0xFFFF0000) >> 16) | ((u32 & 0x0000FFFF) << 16);
    u32 = ((u32 & 0xFF00FF00) >>  8) | ((u32 & 0x00FF00FF) <<  8);
  }
  return u32;
}

typedef enum {
  TAS5828M_RESET_CTRL                = 0x01,
  TAS5828M_DEVICE_CTRL1              = 0x02,
  TAS5828M_DEVICE_CTRL2              = 0x03,
  TAS5828M_PVDD_DROP_DETECTION_CTRL1 = 0x04,
  TAS5828M_PVDD_DROP_DETECTION_CTRL2 = 0x05,
  TAS5828M_I2C_PAGE_AUTO_INC         = 0x0F,
  TAS5828M_SIG_CH_CTRL               = 0x28,
  TAS5828M_CLOCK_DET_CTRL            = 0x29,
  TAS5828M_SDOUT_SEL                 = 0x30,
  TAS5828M_I2S_CTRL                  = 0x31,
  TAS5828M_SAP_CTRL1                 = 0x33,
  TAS5828M_SAP_CTRL2                 = 0x34,
  TAS5828M_SAP_CTRL3                 = 0x35,
  TAS5828M_FS_MON                    = 0x37,
  TAS5828M_BCK_MON                   = 0x38,
  TAS5828M_CLKDET_STATUS             = 0x39,
  TAS5828M_DSP_PGM_MODE              = 0x40,
  TAS5828M_GPIO_OD                   = 0x41, // 7:GPIO0, 6:GPIO2, 5:GPIO1, 1=OD/0=PP, other bits=1
  TAS5828M_DSP_CTRL                  = 0x46,
  TAS5828M_DAC_GAIN                  = 0x4C,
  TAS5828M_DIG_VOL_CTRL1             = 0x4E,
  TAS5828M_DIG_VOL_CTRL2             = 0x4F,
  TAS5828M_AUTO_MUTE_CTRL            = 0x50,
  TAS5828M_AUTO_MUTE_TIME            = 0x51,
  TAS5828M_ANA_CTRL                  = 0x53,
  TAS5828M_AGAIN                     = 0x54,
  TAS5828M_PVDD_ADC                  = 0x5E,
  TAS5828M_GPIO_CTRL                 = 0x60,
  TAS5828M_GPIO1_SEL                 = 0x61,
  TAS5828M_GPIO2_SEL                 = 0x62,
  TAS5828M_GPIO0_SEL                 = 0x63,
  TAS5828M_GPIO_INPUT_SEL            = 0x64,
  TAS5828M_GPIO_OUT                  = 0x65,
  TAS5828M_GPIO_OUT_INV              = 0x66,
  TAS5828M_DIE_ID                    = 0x67,
  TAS5828M_POWER_STATE               = 0x68,
  TAS5828M_AUTOMUTE_STATE            = 0x69,
  TAS5828M_PHASE_CTRL                = 0x6A,
  TAS5828M_SS_CTRL0                  = 0x6B,
  TAS5828M_SS_CTRL1                  = 0x6C,
  TAS5828M_SS_CTRL2                  = 0x6D,
  TAS5828M_SS_CTRL3                  = 0x6E,
  TAS5828M_SS_CTRL4                  = 0x6F,
  TAS5828M_CHAN_FAULT                = 0x70,
  TAS5828M_GLOBAL_FAULT1             = 0x71,
  TAS5828M_GLOBAL_FAULT2             = 0x72,
  TAS5828M_WARNING                   = 0x73,
  TAS5828M_PIN_CONTROL1              = 0x74,
  TAS5828M_PIN_CONTROL2              = 0x75,
  TAS5828M_MISC_CONTROL              = 0x76,
  TAS5828M_CBC_CONTROL               = 0x77,
  TAS5828M_FAULT_CLEAR               = 0x78,
} tas5828m_register;

// DSP: set book/page helper
void TAS5828M_Driver::setBookPage(uint8_t book, uint8_t page) {
  write(0, 0);
  write(0x7F, book);
  write(0, page);
}

// DSP: write 32-bit word block for TAS atlas window (0x08..0x7F), wraps to next page at 0x08.
void TAS5828M_Driver::writeDspBlock(uint8_t book, uint8_t page, uint8_t addr, const uint32_t *data, uint8_t words) {
  uint8_t rem_words = words, cur_page = page, cur_addr = addr, idx = 0;
  setBookPage(book, cur_page);
  while(rem_words) {
    if(cur_addr < 0x08 || cur_addr > 0x7F) break;
    uint8_t chunk_words = (0x80 - cur_addr) >> 2;
    if(chunk_words > rem_words) chunk_words = rem_words;

    uint32_t chunk_be[30];
    for(uint8_t n = 0; n < chunk_words; n++) {
      chunk_be[n] = big32(data[idx + n]);
    }
    write(cur_addr, (uint8_t *)chunk_be, chunk_words * 4);

    rem_words -= chunk_words;
    idx += chunk_words;
    if(!rem_words) break;
    cur_page++;
    cur_addr = 0x08;
    setBookPage(book, cur_page);
  }
  setBookPage(0, 0);
}

// Initialize amplifier
void TAS5828M_Driver::init(audio_samplerate samplerate) {
  init(samplerate, TAS5828M_STEREO);
}

void TAS5828M_Driver::init(audio_samplerate samplerate, tas5828m_output output, tas5828m_modulation modulation, tas5828m_pinmode pwm_pinmode) {
  I2C.acquire();
  write(TAS5828M_DIG_VOL_CTRL1,  0x33); // Default ramp: 0.5 dB/step, 1 step per FS ≈ 5.3 ms
  write(TAS5828M_DAC_GAIN,       0xFF); // Set DAC gain (muted)
  delay(6); // allow ramp down
  write(TAS5828M_DEVICE_CTRL2,   0x1A); // DSP reset, Mute, Hi-Z
  delay(100);
  write(TAS5828M_DEVICE_CTRL2,   0x0A); // Mute, Hi-Z
  delay(5);
  write(TAS5828M_RESET_CTRL,     0x11); // Reset digital core, Reset all registers
  delay(100);
  write(TAS5828M_DIG_VOL_CTRL1,  0xFF);
  write(TAS5828M_DAC_GAIN,       0xFF); // Set DAC gain (muted)
  write(TAS5828M_DIG_VOL_CTRL1,  0x33);
  write(TAS5828M_RESET_CTRL,     0x00); // Take out of reset
  delay(5);
  write(TAS5828M_DEVICE_CTRL2,   0x0A); // DSP run, Mute, Hi-Z
  delay(5);
  write(TAS5828M_SAP_CTRL1,      0x00); // 16 bits, I2S
  uint16_t fsmode = 0b1001; // 48K
  if(samplerate == SR_32K ) fsmode = 0b0110;
  if(samplerate == SR_44K1) fsmode = 0b1000;
  if(samplerate == SR_96K ) fsmode = 0b1011;
  write(TAS5828M_SIG_CH_CTRL, 0x00 | (fsmode << 0)); // 32 bits/frame, samplerate kHz
  //write(TAS5828M_CLOCK_DET_CTRL, 0x7C); // Ignore clock errors
  write(TAS5828M_DEVICE_CTRL1, (output << 2) | (modulation & 3)); // Set PBTL(mono) mode, 384KHz FSW, BD/1SPW/HYBRID mode
  if(modulation == TAS5828M_HYBRID) {
    write(TAS5828M_GPIO1_SEL, 0b1101);     // GPIO1 as Class-H (Hybrid Mode)
    if(pwm_pinmode & TAS5828M_INVERTED)
      write(TAS5828M_GPIO_OUT_INV, 0b001); // GPIO1 is inverted
    if(pwm_pinmode & TAS5828M_OPENDRAIN)
      write(TAS5828M_GPIO_OD, 0b00111111); // GPIO1 is open-drain
  }
  write(TAS5828M_GPIO_CTRL, 0b001);        // GPIO1 is output
  delay(5);
  write(TAS5828M_FAULT_CLEAR,    0x80); // Clear sticky faults
  write(TAS5828M_DEVICE_CTRL2,   0x03); // Play
  _id = 0;
  read(TAS5828M_DIE_ID, &_id, 1);          // Fetch DIE_ID
  I2C.release();
}

void TAS5828M_Driver::analogGainRaw(uint8_t raw) {
  write(TAS5828M_AGAIN, raw);
}

void TAS5828M_Driver::analogGain(float gain_db) {
  if(!isfinite(gain_db) || gain_db < -15.5) {
    gain_db = -15.5;
  } else if(gain_db > 0) {
    gain_db = 0;
  }
  analogGainRaw(-(int)roundf(gain_db * 2));
}

void TAS5828M_Driver::gainRaw(uint8_t raw) {
  write(TAS5828M_DAC_GAIN, raw);
}

// Set digital gain in dB (-103 to +24)
void TAS5828M_Driver::gain(float gain_db) {
  uint8_t _gain;
  if(!isfinite(gain_db) || gain_db < -103.5) {
    _gain = 0xFF;
  } else if(gain_db > 24) {
    _gain = 0x00;
  } else {
    _gain = 0x30 - (int)roundf(gain_db * 2);
  }
  gainRaw(_gain);
}

void TAS5828M_Driver::volumeRamp(tas5828m_ramp_speed down_speed, tas5828m_ramp_step down_step, tas5828m_ramp_speed up_speed, tas5828m_ramp_step up_step) {
  uint8_t ctrl1 = (((uint8_t)down_speed & 0x3) << 6)
                | (((uint8_t)down_step & 0x3) << 4)
                | (((uint8_t)up_speed & 0x3) << 2)
                | (((uint8_t)up_step & 0x3) << 0);
  write(TAS5828M_DIG_VOL_CTRL1, ctrl1);
}

void TAS5828M_Driver::volumeEmergencyRamp(tas5828m_ramp_speed down_speed, tas5828m_ramp_step down_step) {
  uint8_t ctrl2 = (((uint8_t)down_speed & 0x3) << 6)
                | (((uint8_t)down_step & 0x3) << 4);
  write(TAS5828M_DIG_VOL_CTRL2, ctrl2);
}

void TAS5828M_Driver::autoMute(bool left_enable, bool right_enable, bool both_channels_together, tas5828m_automute_time left_time, tas5828m_automute_time right_time) {
  uint8_t ctrl = (left_enable ? 0x01 : 0) | (right_enable ? 0x02 : 0) | (both_channels_together ? 0x04 : 0);
  uint8_t time = (((uint8_t)left_time & 0x7) << 4) | (((uint8_t)right_time & 0x7) << 0);
  write(TAS5828M_AUTO_MUTE_CTRL, ctrl);
  write(TAS5828M_AUTO_MUTE_TIME, time);
}

void TAS5828M_Driver::volumeRaw(int32_t l_vol, int32_t r_vol) {
  I2C.acquire();
  uint32_t data[2] = { big32((uint32_t)l_vol), big32((uint32_t)r_vol) };
  setBookPage(0x8C, 0x06);
  write(0x64, (uint8_t *)data, 8);
  setBookPage(0, 0);
  I2C.release();
}

// Set DAC volume registers in dB (mute < -140 to +48)
void TAS5828M_Driver::volume(float l_gain_db, float r_gain_db) {
  if(l_gain_db > 48) l_gain_db = 48;
  float lgain = powf(10.0f, l_gain_db / 20.0f), rgain = lgain;
  if(isfinite(r_gain_db)) {
    if(r_gain_db > 48) r_gain_db = 48;
    rgain = powf(10.0f, r_gain_db / 20.0f);
  }
  TAS5828M_Driver::volumeRaw((int32_t)roundf(lgain * (float)(1u << 23)), (int32_t)roundf(rgain * (float)(1u << 23)));
}

void TAS5828M_Driver::pathSelect(audio_channel left, audio_channel right) {
  write(TAS5828M_SAP_CTRL3, (left << 4) | right); 
}

tas5828m_state TAS5828M_Driver::state() {
  uint8_t state = 0;
  read(TAS5828M_POWER_STATE, &state, 1);
  return (tas5828m_state)state;
}

void TAS5828M_Driver::setState(tas5828m_state state) {
  I2C.acquire();
  uint8_t reg = 0;
  read(TAS5828M_DEVICE_CTRL2, &reg, 1);
  reg = (reg & ~0x03) | ((uint8_t)state & 0x03);
  write(TAS5828M_DEVICE_CTRL2, reg);
  I2C.release();
}

tas5828m_clockmon_t TAS5828M_Driver::clockMon() {
  tas5828m_clockmon_t mon = {};
  read(TAS5828M_FS_MON, (uint8_t *)&mon.raw, 3);
  return mon;
}

audio_channel TAS5828M_Driver::autoMuteState() {
  uint8_t raw = 0;
  read(TAS5828M_AUTOMUTE_STATE, &raw, 1);
  return (audio_channel)(raw & CH_BOTH);
}

tas5828m_status_t TAS5828M_Driver::status() {
  tas5828m_status_t status = {};
  read(TAS5828M_CHAN_FAULT, (uint8_t *)&status.raw, 4);
  return status;
}

void TAS5828M_Driver::clearFaults() {
  write(TAS5828M_FAULT_CLEAR, 0x80);
}

uint8_t TAS5828M_Driver::pvddRaw() {
  uint8_t raw = 0;
  read(TAS5828M_PVDD_ADC, &raw, 1);
  return raw;
}

float TAS5828M_Driver::pvdd() {
  uint8_t rpt = pvddRaw();
  return rpt * 0.12f;
}

void TAS5828M_Driver::biquadSet(audio_channel ch, uint8_t stage, biquad_filter_q &filter) {
  if(stage >= TAS5828M_BQ_COUNT) return;
  I2C.acquire();

  // DSP atlas                  BQ                                                                       PostEQ1      PostEQ2
  const uint8_t bqAddrL[16] = { 0x30, 0x44, 0x58, 0x6C, 0x08, 0x1C, 0x30, 0x44, 0x58, 0x6C, 0x08, 0x1C,  0x30, 0x44,  0x08, 0x1C };
  const uint8_t bqBookL[16] = { 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x03, 0x03,  0x05, 0x05,  0x06, 0x06 };
  const uint8_t bqAddrR[16] = { 0x30, 0x44, 0x58, 0x6C, 0x08, 0x1C, 0x30, 0x44, 0x58, 0x6C, 0x08, 0x1C,  0x58, 0x6C,  0x30, 0x44 };
  const uint8_t bqBookR[16] = { 0x03, 0x03, 0x03, 0x03, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x05, 0x05,  0x05, 0x05,  0x06, 0x06 };

  if(ch & 1) {
    setBookPage(0, 0); // Undocumented but possibly necessary for live update (research required)
    write(0x5C, 1);
    writeDspBlock(0xAA, bqBookL[stage], bqAddrL[stage], (const uint32_t *)filter.coef, 5);
  }
  if(ch & 2) {
    setBookPage(0, 0); // Undocumented but possibly necessary for live update (research required)
    write(0x5C, 1);
    writeDspBlock(0xAA, bqBookR[stage], bqAddrR[stage], (const uint32_t *)filter.coef, 5);
  }
  I2C.release();
}

void TAS5828M_Driver::biquadSet(audio_channel ch, uint8_t stage, biquad_filter &filter) {
  if(stage >= TAS5828M_BQ_COUNT) return;
  biquad_filter_q q = biquadQ(&filter, TAS5828M_BQ_Q);
  biquadSet(ch, stage, q);
}

void TAS5828M_Driver::inputSetupRaw(uint32_t l_from_l, uint32_t l_from_r, uint32_t r_from_l, uint32_t r_from_r) {
  I2C.acquire();
  uint32_t data[4] = {
    l_from_l, // 0: Input mixer left from left (Q9.23)
    l_from_r, // 1: Input mixer left from right (Q9.23)
    r_from_l, // 2: Input mixer right from left (Q9.23)
    r_from_r  // 3: Input mixer right from right (Q9.23)
  };
  writeDspBlock(0x8C, 0x09, 0x74, data, 4);
  I2C.release();
}

void TAS5828M_Driver::inputSetup(float l_from_l, float l_from_r, float r_from_l, float r_from_r) {
  // Input mixer 2x2 (Q9.23):
  // outL = inL*l_from_l + inR*l_from_r, outR = inL*r_from_l + inR*r_from_r.
  inputSetupRaw(
    (uint32_t)floatQ(l_from_l, 23), // Input Mixer Left from Left (Q9.23)
    (uint32_t)floatQ(l_from_r, 23), // Input Mixer Left from Right (Q9.23)
    (uint32_t)floatQ(r_from_l, 23), // Input Mixer Right from Left (Q9.23)
    (uint32_t)floatQ(r_from_r, 23)  // Input Mixer Right from Right (Q9.23)
  );
}

void TAS5828M_Driver::crossbarSetupRaw(uint32_t amp_l_from_l, uint32_t amp_l_from_r, uint32_t amp_r_from_l, uint32_t amp_r_from_r, uint32_t i2s_l_from_l, uint32_t i2s_l_from_r, uint32_t i2s_r_from_l, uint32_t i2s_r_from_r) {
  I2C.acquire();
  uint32_t data[8] = {
    i2s_l_from_l, // 0: I2S left from left (Q9.23)
    i2s_l_from_r, // 1: I2S left from right (Q9.23)
    i2s_r_from_l, // 2: I2S right from left (Q9.23)
    i2s_r_from_r, // 3: I2S right from right (Q9.23)
    amp_l_from_l, // 4: Amp left from left (Q9.23)
    amp_l_from_r, // 5: Amp left from right (Q9.23)
    amp_r_from_l, // 6: Amp right from left (Q9.23)
    amp_r_from_r  // 7: Amp right from right (Q9.23)
  };
  writeDspBlock(0x8C, 0x0A, 0x0C, &data[0], 2);
  writeDspBlock(0x8C, 0x0A, 0x1C, &data[2], 2);
  writeDspBlock(0x8C, 0x0A, 0x24, &data[4], 2);
  writeDspBlock(0x8C, 0x0A, 0x30, &data[6], 2);
  I2C.release();
}

void TAS5828M_Driver::crossbarSetup(float amp_l_from_l, float amp_l_from_r, float amp_r_from_l, float amp_r_from_r, float i2s_l_from_l, float i2s_l_from_r, float i2s_r_from_l, float i2s_r_from_r) {
  // Output crossbar 2x2 + 2x2 (Q9.23): amp path and I2S path.
  crossbarSetupRaw(
    (uint32_t)floatQ(amp_l_from_l, 23), // Amp Left from Left (Q9.23)
    (uint32_t)floatQ(amp_l_from_r, 23), // Amp Left from Right (Q9.23)
    (uint32_t)floatQ(amp_r_from_l, 23), // Amp Right from Left (Q9.23)
    (uint32_t)floatQ(amp_r_from_r, 23), // Amp Right from Right (Q9.23)
    (uint32_t)floatQ(i2s_l_from_l, 23), // I2S Left from Left (Q9.23)
    (uint32_t)floatQ(i2s_l_from_r, 23), // I2S Left from Right (Q9.23)
    (uint32_t)floatQ(i2s_r_from_l, 23), // I2S Right from Left (Q9.23)
    (uint32_t)floatQ(i2s_r_from_r, 23)  // I2S Right from Right (Q9.23)
  );
}

void TAS5828M_Driver::clipperSetupRaw(uint32_t prescale, uint32_t l_postscale, uint32_t r_postscale) {
  I2C.acquire();
  uint32_t data[3] = {
    prescale,    // 0: CH-LR THD boost (reality says Q5.27, datasheet says Q9.23)
    l_postscale, // 1: CH-L fine volume (reality says Q6.26, datasheet says Q1.31)
    r_postscale  // 2: CH-R fine volume (reality says Q6.26, datasheet says Q1.31)
  };
  writeDspBlock(0x8C, 0x0A, 0x3C, data, 3);
  I2C.release();
}

void TAS5828M_Driver::clipperSetup(float prescale, float l_postscale, float r_postscale) {
  float inv_prescale = (isfinite(prescale) && prescale != 0.0f) ? (1.0f / prescale) : 1.0f;
  if(isnan(l_postscale)) l_postscale = inv_prescale;
  if(isnan(r_postscale)) r_postscale = l_postscale;
  clipperSetupRaw(
    (uint32_t)floatQ(prescale, 27), // Gain into clip region (reality says Q5.27, datasheet says Q9.23)
    (uint32_t)floatQ(l_postscale, 26), // Left recovery gain (reality says Q6.26, datasheet says Q1.31)
    (uint32_t)floatQ(r_postscale, 26) // Right recovery gain (reality says Q6.26, datasheet says Q1.31)
  );
}

void TAS5828M_Driver::dpeqSetupRaw(uint32_t alpha, uint32_t gain, uint32_t offset) {
  I2C.acquire();
  uint32_t data[3] = {
    alpha,  // 0: DPEQ sense alpha (Q1.31)
    gain,   // 1: DPEQ threshold gain (reality Q5.27 specification Q1.31)
    offset  // 2: DPEQ threshold offset (Q1.31)
  };
  writeDspBlock(0x8C, 0x07, 0x34, data, 3);
  I2C.release();
}

void TAS5828M_Driver::dpeqSetup(float alpha, float gain, float offset) {
  dpeqSetupRaw(
    (uint32_t)floatQ(alpha, 31),
    (uint32_t)floatQ(gain, 27),
    (uint32_t)floatQ(offset, 31)
  );
}

void TAS5828M_Driver::dpeqBiquadSet(uint8_t stage, biquad_filter_q &filter) {
  if(stage > 2) return;
  // DSP atlas                  SENSE LOW   HIGH
  const uint8_t dpeqAddr[3] = { 0x70, 0x0C, 0x20 };
  const uint8_t dpeqBook[3] = { 0x06, 0x07, 0x07 };
  I2C.acquire();
  setBookPage(0, 0); // Undocumented but possibly necessary for live update (research required)
  write(0x5C, 1);
  writeDspBlock(0xAA, dpeqBook[stage], dpeqAddr[stage], (const uint32_t *)filter.coef, 5);
  I2C.release();
}

void TAS5828M_Driver::dpeqBiquadSet(uint8_t stage, biquad_filter &filter) {
  biquad_filter_q q = biquadQ(&filter, TAS5828M_BQ_Q);
  dpeqBiquadSet(stage, q);
}

void TAS5828M_Driver::drc3TimeSetupRaw(uint8_t band, uint32_t alpha, uint32_t attack, uint32_t decay) {
  if(band > 2) return;
  // DSP atlas                 DRC1  DRC2  DRC3
  const uint8_t drcAddr[3] = { 0x68, 0x40, 0x18 };
  const uint8_t drcBook[3] = { 0x08, 0x09, 0x09 };
  I2C.acquire();
  uint32_t data[3] = {
    alpha,  // 0: DRCx alpha (Q1.31)
    attack, // 1: DRCx attack (Q1.31)
    decay   // 2: DRCx decay (Q1.31)
  };
  writeDspBlock(0x8C, drcBook[band], drcAddr[band], data, 3);
  I2C.release();
}

void TAS5828M_Driver::drc3TimeSetup(uint8_t band, float alpha, float attack, float decay) {
  drc3TimeSetupRaw(
    band,
    (uint32_t)floatQ(alpha, 31),
    (uint32_t)floatQ(attack, 31),
    (uint32_t)floatQ(decay, 31)
  );
}

void TAS5828M_Driver::drc3CurveSetupRaw(uint8_t band, uint32_t k0, uint32_t k1, uint32_t k2, uint32_t t1, uint32_t t2, uint32_t off1, uint32_t off2) {
  if(band > 2) return;
  // DSP atlas                 DRC1  DRC2  DRC3
  const uint8_t drcAddr[3] = { 0x74, 0x4C, 0x24 };
  const uint8_t drcBook[3] = { 0x08, 0x09, 0x09 };
  I2C.acquire();
  uint32_t data[7] = {
    k0,   // 0: DRCx Region 0 Slope (Q9.23)
    k1,   // 1: DRCx Region 1 Slope (Q9.23)
    k2,   // 2: DRCx Region 2 Slope (Q9.23)
    t1,   // 3: DRCx Region 1 Threshold (Q9.23)
    t2,   // 4: DRCx Region 2 Threshold (Q9.23)
    off1, // 5: DRCx Region 1 Offset (Q9.23)
    off2  // 6: DRCx Region 2 Offset (Q9.23)
  };

  writeDspBlock(0x8C, drcBook[band], drcAddr[band], data, 7);
  I2C.release();
}

void TAS5828M_Driver::drc3CurveSetup(uint8_t band, float k0, float k1, float k2, float t1, float t2, float off1, float off2) {
  drc3CurveSetupRaw(
    band,
    (uint32_t)floatQ(k0, 23),
    (uint32_t)floatQ(k1, 23),
    (uint32_t)floatQ(k2, 23),
    (uint32_t)floatQ(t1, 23),
    (uint32_t)floatQ(t2, 23),
    (uint32_t)floatQ(off1, 23),
    (uint32_t)floatQ(off2, 23)
  );
}

void TAS5828M_Driver::drc3MixerSetupRaw(uint32_t drc1, uint32_t drc2, uint32_t drc3) {
  I2C.acquire();
  uint32_t data[3] = {
    drc1, // 0: DRC1 mixer gain (Q9.23)
    drc3, // 1: DRC3 mixer gain (Q9.23)
    drc2  // 2: DRC2 mixer gain (Q9.23)
  };
  writeDspBlock(0x8C, 0x09, 0x68, data, 3);
  I2C.release();
}

void TAS5828M_Driver::drc3MixerSetup(float drc1, float drc2, float drc3) {
  drc3MixerSetupRaw(
    (uint32_t)floatQ(drc1, 23),
    (uint32_t)floatQ(drc2, 23),
    (uint32_t)floatQ(drc3, 23)
  );
}

void TAS5828M_Driver::drc3BiquadSet(uint8_t stage, biquad_filter_q &filter) {
  if(stage > 7) return;
  // DSP atlas                 LOW0  LOW1    MID0  MID1  MID2  MID3    HIGH0 HIGH1
  const uint8_t drcAddr[8] = { 0x40, 0x54,   0x18, 0x2C, 0x40, 0x54,   0x68, 0x7C };
  const uint8_t drcBook[8] = { 0x07, 0x07,   0x08, 0x08, 0x08, 0x08,   0x07, 0x07 };
  I2C.acquire();
  setBookPage(0, 0); // Undocumented but possibly necessary for live update (research required)
  write(0x5C, 1);
  writeDspBlock(0xAA, drcBook[stage], drcAddr[stage], (const uint32_t *)filter.coef, 5);
  I2C.release();
}

void TAS5828M_Driver::drc3BiquadSet(uint8_t stage, biquad_filter &filter) {
  biquad_filter_q q = biquadQ(&filter, TAS5828M_BQ_Q);
  drc3BiquadSet(stage, q);
}

void TAS5828M_Driver::aglSetupRaw(uint32_t alpha, uint32_t attack, uint32_t enable, uint32_t omega, uint32_t decay, uint32_t threshold, uint32_t temp_scale, uint32_t volt_scale, uint32_t volt_temp_scale) {
  I2C.acquire();
  uint32_t data[9] = {
    alpha,          // 0: Alpha (Q1.31)
    attack,         // 1: Attack (Q1.31)
    enable,         // 2: Enable flags
    omega,          // 3: Omega (Q1.31)
    decay,          // 4: Decay (Q1.31)
    threshold,      // 5: Attack threshold (Q1.31)
    temp_scale,     // 6: Temp scale (Q9.23)
    volt_scale,     // 7: Volt scale (Q9.23)
    volt_temp_scale // 8: Volt/temp scale (Q9.23)
  };
  writeDspBlock(0x8C, 0x0C, 0x08, &data[0], 3); // alpha, attack, enable
  writeDspBlock(0x8C, 0x0C, 0x18, &data[3], 2); // omega, decay
  writeDspBlock(0x8C, 0x0C, 0x24, &data[5], 1); // threshold
  writeDspBlock(0x8C, 0x0C, 0x28, &data[6], 1); // temp_scale
  writeDspBlock(0x8C, 0x0C, 0x30, &data[7], 2); // volt_scale, volt_temp_scale
  I2C.release();
}

void TAS5828M_Driver::aglSetup(bool enable, float threshold, float attack, float decay, float alpha, float omega, float temp_scale, float volt_scale, float volt_temp_scale) {
  aglSetupRaw(
    (uint32_t)floatQ(alpha, 31),
    (uint32_t)floatQ(attack, 31),
    enable ? 0xC0000000u : 0x00000000u,
    (uint32_t)floatQ(omega, 31),
    (uint32_t)floatQ(decay, 31),
    (uint32_t)floatQ(threshold, 31),
    (uint32_t)floatQ(temp_scale, 23),
    (uint32_t)floatQ(volt_scale, 23),
    (uint32_t)floatQ(volt_temp_scale, 23)
  );
}

void TAS5828M_Driver::hybridSetupRaw(uint32_t l_delay_samples, uint32_t r_delay_samples, uint32_t window_samples, uint32_t hold_samples, uint32_t peak_offset, uint32_t peak_decay, uint32_t peak_smooth, uint32_t threshold) {
  I2C.acquire();
  uint32_t data[9] = {
    0,               // 0: Class-H bypass flag (0 = enabled)
    l_delay_samples, // 1: Left delay samples
    r_delay_samples, // 2: Right delay samples
    window_samples,  // 3: Max detect window
    hold_samples,    // 4: Peak hold samples
    peak_offset,     // 5: Peak detect offset (Q1.31)
    peak_decay,      // 6: Peak decay (Q1.31)
    peak_smooth,     // 7: Peak smooth coefficient (Q1.31)
    threshold        // 8: Class-H threshold (Q5.27)
  };

  writeDspBlock(0x8C, 0x0A, 0x5C, &data[1], 2); // delay left/right
  writeDspBlock(0x8C, 0x0A, 0x68, &data[3], 6); // window..threshold
  writeDspBlock(0x8C, 0x0A, 0x50, &data[0], 1); // class-H bypass
  I2C.release();
}

void TAS5828M_Driver::hybridSetup(uint32_t delay_samples, uint32_t window_samples, uint32_t hold_samples, float peak_offset, float peak_decay, float peak_smooth, float threshold) {
  hybridSetupRaw(
    delay_samples,
    delay_samples,
    window_samples,
    hold_samples,
    (uint32_t)floatQ(peak_offset, 31),
    (uint32_t)floatQ(peak_decay, 31),
    (uint32_t)floatQ(peak_smooth, 31),
    (uint32_t)floatQ(threshold, 27)
  );
}
