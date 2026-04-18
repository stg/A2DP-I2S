// SGTL5000 is an Audio_Codec driver for the chip with the same name

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include "I2C.h"
#include "SGTL5000.h"

// This SGTL5000 driver was reimplemented for this project and platform with reference to the SGTL5000
// datasheet and PJRC's Teensy Audio SGTL5000 driver. This file is part of A2DP-I2S and is licensed
// under this project's license; it does not incorporate PJRC source. Credit is due to PJRC for their
// work on the Teensy Audio Library, which served as a useful reference during development.

// Abstraction
uint16_t SGTL5000_Driver::write(uint16_t reg, uint16_t data) { I2C.write16(_address, reg, data); return data; }
uint16_t SGTL5000_Driver::read(uint16_t reg) { uint8_t u8[2] = {}; I2C.read(_address, reg, u8, 2, true); return (u8[0] << 8) | u8[1]; }
void SGTL5000_Driver::delay(int time) { vTaskDelay(time / portTICK_PERIOD_MS); }
void SGTL5000_Driver::writeDapBlock(uint16_t start_reg, const uint16_t *data, uint8_t count) {
	for(uint8_t i = 0; i < count; ++i) {
		write(start_reg + (i * 2), data[i]);
	}
}

// Common rounded scaling calculation
static inline uint8_t scale_code(float value, float multiplier, float offset, uint8_t min_code, uint8_t max_code) {
	float code = value * multiplier + offset;
	if(code < min_code) return min_code;
	if(code > max_code) return max_code;
	return (uint8_t)lroundf(code);
}

typedef enum {
	SGTL5000_ID                        = 0x0000,
	SGTL5000_DIG_POWER                 = 0x0002,
	SGTL5000_CLK_CTRL                  = 0x0004,
	SGTL5000_I2S_CTRL                  = 0x0006,
	SGTL5000_SSS_CTRL                  = 0x000A,
	SGTL5000_ADCDAC_CTRL               = 0x000E,
	SGTL5000_DAC_VOL                   = 0x0010,
	SGTL5000_PAD_STRENGTH              = 0x0014,
	SGTL5000_ANA_ADC_CTRL              = 0x0020,
	SGTL5000_ANA_HP_CTRL               = 0x0022,
	SGTL5000_ANA_CTRL                  = 0x0024,
	SGTL5000_LINREG_CTRL               = 0x0026,
	SGTL5000_REF_CTRL                  = 0x0028,
	SGTL5000_MIC_CTRL                  = 0x002A,
	SGTL5000_LINE_OUT_CTRL             = 0x002C,
	SGTL5000_LINE_OUT_VOL              = 0x002E,
	SGTL5000_ANA_POWER                 = 0x0030,
	SGTL5000_PLL_CTRL                  = 0x0032,
	SGTL5000_CLK_TOP_CTRL              = 0x0034,
	SGTL5000_ANA_STATUS                = 0x0036,
	SGTL5000_SHORT_CTRL                = 0x003C,
	SGTL5000_DAP_CONTROL               = 0x0100,
	SGTL5000_DAP_PEQ                   = 0x0102,
	SGTL5000_DAP_BASS_ENHANCE          = 0x0104,
	SGTL5000_DAP_BASS_ENHANCE_CTRL     = 0x0106,
	SGTL5000_DAP_AUDIO_EQ              = 0x0108,
	SGTL5000_DAP_SGTL_SURROUND         = 0x010A,
	SGTL5000_DAP_FILTER_COEF_ACCESS    = 0x010C,
	SGTL5000_DAP_COEF_WR_B0_MSB        = 0x010E,
	SGTL5000_DAP_COEF_WR_B0_LSB        = 0x0110,
	SGTL5000_DAP_AUDIO_EQ_BASS_BAND0   = 0x0116, // GEQ band 0 center / low crossover ~115 Hz
	SGTL5000_DAP_AUDIO_EQ_BAND1        = 0x0118, // GEQ band 1 center ~330 Hz
	SGTL5000_DAP_AUDIO_EQ_BAND2        = 0x011A, // GEQ band 2 center ~990 Hz
	SGTL5000_DAP_AUDIO_EQ_BAND3        = 0x011C, // GEQ band 3 center ~3.0 kHz
	SGTL5000_DAP_AUDIO_EQ_TREBLE_BAND4 = 0x011E, // GEQ band 4 center / high crossover ~9.9 kHz
	SGTL5000_DAP_MAIN_CHAN             = 0x0120,
	SGTL5000_DAP_MIX_CHAN              = 0x0122,
	SGTL5000_DAP_AVC_CTRL              = 0x0124,
	SGTL5000_DAP_AVC_THRESHOLD         = 0x0126,
	SGTL5000_DAP_AVC_ATTACK            = 0x0128,
	SGTL5000_DAP_AVC_DECAY             = 0x012A,
	SGTL5000_DAP_COEF_WR_B1_MSB        = 0x012C,
	SGTL5000_DAP_COEF_WR_B1_LSB        = 0x012E,
	SGTL5000_DAP_COEF_WR_B2_MSB        = 0x0130,
	SGTL5000_DAP_COEF_WR_B2_LSB        = 0x0132,
	SGTL5000_DAP_COEF_WR_A1_MSB        = 0x0134,
	SGTL5000_DAP_COEF_WR_A1_LSB        = 0x0136,
	SGTL5000_DAP_COEF_WR_A2_MSB        = 0x0138,
	SGTL5000_DAP_COEF_WR_A2_LSB        = 0x013A,
} sgtl5000_register;

// Initialize Codec
void SGTL5000_Driver::init(audio_samplerate samplerate) {
	I2C.acquire();
	// Hold the playback path quiet before changing analog power and routing.
	write(SGTL5000_ADCDAC_CTRL,   0x000C); // DAC mute L+R
	write(SGTL5000_ANA_HP_CTRL,   0x7F7F); // HP gain minimum
	write(SGTL5000_DAC_VOL,       0xFCFC); // Digital playback mute
	delay(25);
	// Establish the analog supply, reference, and output protection profile.
	write(SGTL5000_ANA_POWER,     0x4060); // External VDDD, stereo ADC path, reference bias on
	write(SGTL5000_LINREG_CTRL,   0x006C); // VDDIO-assigned charge pump, >3.1 V rails
	write(SGTL5000_REF_CTRL,      0x01F2); // VAG=1.575 V, +12.5% bias, normal ramp
	write(SGTL5000_LINE_OUT_CTRL, 0x0F1F); // LO_VAG=1.575 V, max line-out bias current
	write(SGTL5000_SHORT_CTRL,    0x5556); // LR short trip = 150 mA
	write(SGTL5000_ANA_CTRL,      0x0137); // Zero-cross detect active, analog path held muted
	write(SGTL5000_ANA_POWER,     0x40FF); // Power lineout, HP, ADC, DAC, reference
	write(SGTL5000_DIG_POWER,     0x0073); // Power I2S in/out, DAP, DAC, ADC
	delay(100);
	write(SGTL5000_MIC_CTRL,      0x0000); // Mic off: no bias, min gain
	write(SGTL5000_ANA_ADC_CTRL,  0x0000); // line/mic ADC PGA = 0 dB default
	write(SGTL5000_LINE_OUT_VOL,  0x0F0F); // Line-out nominal 0 dB when LO_VAG == VAG
	uint16_t sys_fs = 0x2; // Sample rate
	if(samplerate == SR_32K ) sys_fs = 0;
	if(samplerate == SR_44K1) sys_fs = 1;
	if(samplerate == SR_96K ) sys_fs = 3;
	// Configure clocks, serial audio format, and default signal routing.
	write(SGTL5000_CLK_CTRL,      0x0004 | (sys_fs << 2)); // SYS_FS = samplerate, MCLK = 256xFs
	write(SGTL5000_I2S_CTRL,      0x0130); // 16-bit I2S slave timing
	write(SGTL5000_SSS_CTRL,      0x0010); // ADC->I2S, I2S->DAC
	// Default to post-DAC DAP processing with the full PEQ stage count available.
	dapSetup(SGTL5000_DAP_POST);
	eqModeSetup(SGTL5000_EQ_PEQ, 7);
	delay(25);
	write(SGTL5000_ADCDAC_CTRL,   0x0000); // DAC unmute
	write(SGTL5000_ANA_CTRL,      0x0036); // Zero-cross detect active, playback path unmuted
	_id = read(SGTL5000_ID);               // Cache chip ID
	I2C.release();
}

void SGTL5000_Driver::hpVolumeRaw(uint8_t left, uint8_t right) {
	write(SGTL5000_ANA_HP_CTRL, ((uint16_t)right << 8) | left);
}

void SGTL5000_Driver::hpVolume(float left_norm, float right_norm) {
	hpVolumeRaw(scale_code(left_norm, -0x7F, 0x7F, 0, 0x7F), scale_code(right_norm, -0x7F, 0x7F, 0, 0x7F));
}

void SGTL5000_Driver::hpVolume(float vol_norm) {
  const uint8_t raw = scale_code(vol_norm, -0x7F, 0x7F, 0, 0x7F);
  hpVolumeRaw(raw, raw);
}

void SGTL5000_Driver::lineOutLevel(uint8_t level) {
	if(level > 31) level = 31;
	else if(level < 13) level = 13;
	write(SGTL5000_LINE_OUT_VOL, (level << 8) | level);
}

void SGTL5000_Driver::lineOutLevel(uint8_t left, uint8_t right) {
	if(left > 31) left = 31;
	else if(left < 13) left = 13;
	if(right > 31) right = 31;
	else if(right < 13) right = 13;
	write(SGTL5000_LINE_OUT_VOL, ((uint16_t)right << 8) | left);
}

void SGTL5000_Driver::lineInGain(uint8_t level) {
	lineInGain(level, level);
}

void SGTL5000_Driver::lineInGain(uint8_t left, uint8_t right) {
	if(left > 15) left = 15;
	if(right > 15) right = 15;
	write(SGTL5000_ANA_ADC_CTRL, (left << 4) | right);
}

void SGTL5000_Driver::micSetupRaw(uint16_t mic_ctrl, uint16_t ana_adc_ctrl) {
	write(SGTL5000_MIC_CTRL, mic_ctrl);
	write(SGTL5000_ANA_ADC_CTRL, ana_adc_ctrl);
}

void SGTL5000_Driver::micSetup(unsigned int gain_db, float bias_v) {
	unsigned int preamp_gain = 0;
	unsigned int bias_resistor = 0, bias_voltage = 0;
	if(gain_db >= 20) {
		preamp_gain = 1 + ((gain_db - 20) / 10);
		if(preamp_gain > 3) preamp_gain = 3;
	}
	unsigned int preamp_db = (preamp_gain == 0) ? 0 : (preamp_gain * 10 + 10);
	unsigned int input_gain = (gain_db - preamp_db) * 2 / 3;
	if(input_gain > 15) input_gain = 15;

	if(bias_v > 0) {
		bias_resistor = 1 << 8; 
		bias_voltage = (uint16_t)lroundf((bias_v - 1.25f) / 0.25f) << 4;
	}

	micSetupRaw(0x0170 | preamp_gain | bias_voltage | bias_resistor, (input_gain << 4) | input_gain);
}

void SGTL5000_Driver::analogPathSetup(sgtl5000_input input, sgtl5000_headphone headphone, bool headphone_mute, bool lineout_mute) {
	I2C.acquire();
	uint16_t ana_ctrl = 0x0026;
	if(input == SGTL5000_IN_LINEIN) ana_ctrl |= (1 << 2);
	if(headphone == SGTL5000_HP_DAC) ana_ctrl |= (1 << 6);
	if(headphone_mute) ana_ctrl |= (1 << 4);
	if(lineout_mute) ana_ctrl |= (1 << 8);
	write(SGTL5000_ANA_CTRL, ana_ctrl);
	I2C.release();
}

void SGTL5000_Driver::volumeRaw(uint8_t l_vol, uint8_t r_vol) {
	write(SGTL5000_DAC_VOL, ((uint16_t)r_vol << 8) | l_vol);
}

void SGTL5000_Driver::volume(float l_gain_db, float r_gain_db) {
	uint8_t l_gain = scale_code(l_gain_db, -2.0f, 0x3C, 0x3C, 0xF1);
	if(l_gain > 0xF0) l_gain = 0xFC;

	uint8_t r_gain = l_gain;
	if(isfinite(r_gain_db)) {
		r_gain = scale_code(r_gain_db, -2.0f, 0x3C, 0x3C, 0xF1);
		if(r_gain > 0xF0) r_gain = 0xFC;
	}
	volumeRaw(l_gain, r_gain);
}

void SGTL5000_Driver::dacPathSetup(sgtl5000_dac_ramp ramp, sgtl5000_hpf hpf, audio_channel mute) {
	uint16_t adcdac = ((uint16_t)(mute & CH_BOTH) << 2);
	if(ramp == SGTL5000_RAMP_EXP) adcdac |= 0x0300;
	else if(ramp == SGTL5000_RAMP_LINEAR) adcdac |= 0x0200;
	if(hpf == SGTL5000_HPF_DISABLE) adcdac |= 0x0001;
	else if(hpf == SGTL5000_HPF_FREEZE) adcdac |= 0x0002;
	write(SGTL5000_ADCDAC_CTRL, adcdac);
}

void SGTL5000_Driver::dapSetup(sgtl5000_dap_mode mode) {
	if(mode == SGTL5000_DAP_PRE) {
		write(SGTL5000_DAP_CONTROL, 1);
		write(SGTL5000_SSS_CTRL, 0x0013);
	} else if(mode == SGTL5000_DAP_POST) {
		write(SGTL5000_DAP_CONTROL, 1);
		write(SGTL5000_SSS_CTRL, 0x0070);
	} else {
		write(SGTL5000_SSS_CTRL, 0x0010);
		write(SGTL5000_DAP_CONTROL, 0);
	}
}

void SGTL5000_Driver::eqModeSetup(sgtl5000_eq eq, uint8_t peqCount) {
	write(SGTL5000_DAP_AUDIO_EQ, (uint16_t)eq & 3);
	write(SGTL5000_DAP_PEQ, (uint16_t)(peqCount & 7));
}

void SGTL5000_Driver::geqBandSetRaw(uint8_t band, uint8_t gain) {
	write(SGTL5000_DAP_AUDIO_EQ_BASS_BAND0 + (2 * band), gain);
}

void SGTL5000_Driver::geqBandSet(uint8_t band, float gain_norm) {
	geqBandSetRaw(band, scale_code(gain_norm, 48, 47, 0, 95));
}

void SGTL5000_Driver::geqSetup(float bass_norm, float mid_bass_norm, float midrange_norm, float mid_treble_norm, float treble_norm) {
	geqBandSet(0, bass_norm);
	geqBandSet(1, mid_bass_norm);
	geqBandSet(2, midrange_norm);
	geqBandSet(3, mid_treble_norm);
	geqBandSet(4, treble_norm);
}

void SGTL5000_Driver::geqSetup(float bass_norm, float treble_norm) {
	geqBandSet(0, bass_norm);
	geqBandSet(4, treble_norm);
}

void SGTL5000_Driver::biquadSet(audio_channel ch, uint8_t stage, biquad_filter &filter) {
	(void)ch;
  if(stage >= SGTL5000_BQ_COUNT) return;
	biquad_filter_q q = biquadQ(&filter, SGTL5000_BQ_Q);
	biquadSet(CH_BOTH, stage, q);
}

void SGTL5000_Driver::biquadSet(audio_channel ch, uint8_t stage, biquad_filter_q &filter) {
	if(stage >= SGTL5000_BQ_COUNT) return;
  (void)ch;
	const int32_t *filterParameters = filter.coef;
	uint16_t block[10];
	for(uint8_t i = 0; i < 5; ++i) {
		block[i * 2 + 0] = (uint16_t)((filterParameters[i] >> 4) & 0xFFFF);
		block[i * 2 + 1] = (uint16_t)(filterParameters[i] & 0xF);
	}
	I2C.acquire();
	write(SGTL5000_DAP_FILTER_COEF_ACCESS, (uint16_t)stage); // Possibly necessary for live update (research required)
	writeDapBlock(SGTL5000_DAP_COEF_WR_B0_MSB, block, 10);
	write(SGTL5000_DAP_FILTER_COEF_ACCESS, (uint16_t)0x100 | stage);
	I2C.release();
}

void SGTL5000_Driver::avcSetupRaw(uint16_t threshold, uint16_t attack, uint16_t decay, uint16_t avc_ctrl) {
	const uint16_t block[] = { avc_ctrl, threshold, attack, decay };
	writeDapBlock(SGTL5000_DAP_AVC_CTRL, block, 4);
}

void SGTL5000_Driver::avcSetup(bool enabled, sgtl5000_avc_max_gain max_gain, sgtl5000_avc_smooth smooth, bool hard_limit, float threshold_db, float attack_dbps, float decay_dbps) {
	uint16_t thresh = powf(10.0f, threshold_db / 20.0f) * 0.636f * 32768.0f;
	uint16_t att = (1.0f - powf(10.0f, -attack_dbps / (20.0f * 44100.0f))) * 524288.0f;
	uint16_t dec = (1.0f - powf(10.0f, -decay_dbps / (20.0f * 44100.0f))) * 8388608.0f;
	avcSetupRaw(thresh, att, dec, (((uint16_t)max_gain & 0x3) << 12) | (((uint16_t)smooth & 0x3) << 8) | ((hard_limit ? 1 : 0) << 5) | (enabled ? 1 : 0));
}

void SGTL5000_Driver::bassEnhanceSetupRaw(uint16_t bass_enhance, uint16_t bass_enhance_ctrl) {
	const uint16_t block[] = { bass_enhance, bass_enhance_ctrl };
	writeDapBlock(SGTL5000_DAP_BASS_ENHANCE, block, 2);
}

void SGTL5000_Driver::bassEnhanceSetup(bool enabled, float lr_mix_norm, float bass_gain_norm, bool hpf_bypass, sgtl5000_cutoff cutoff) {
	bassEnhanceSetupRaw((enabled ? 1 : 0) | (((uint16_t)cutoff & 7) << 4) | ((hpf_bypass ? 1 : 0) << 8),
                     (scale_code(lr_mix_norm, -0x3F, 0x3F, 0, 0x3F) << 8) | scale_code(bass_gain_norm, -0x7F, 0x7F, 0, 0x7F));
}

void SGTL5000_Driver::surroundSetupRaw(uint16_t surround) {
	write(SGTL5000_DAP_SGTL_SURROUND, surround);
}

void SGTL5000_Driver::surroundSetup(bool enabled, uint8_t width, uint8_t select) {
	surroundSetupRaw(((width & 7) << 4) | (enabled ? (select & 3) : 0));
}
