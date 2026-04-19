// Designed for ESP32
// esp32 library version 3.0.7 (recent versions use more ram and will fail)
// pschatzmann/ESP32-A2DP library version 1.8.8

#include "TAS5828M.h"
#include "SGTL5000.h"
#include "Audio.h"
#include "Comm.h"
#include "I2C.h"
#include "Biquad.h"
#include "OTA.h" // Enable OTA
#include "Vibe.h" // Enable Vibe

#define USE_TAS5828M 1
#define HAVE_VOL_POT 1 // else uses tas5828m_gain, sgtl5000_gain

#define DEVICE_NAME "MySpeaker"

/*
TAS5828M
  [X] start in mono (untested)/stereo
  [X] configure biquads
  [X] configure gain chain
  [X] support class-h power supplies
  [X] support swap l/r
  [X] support swap phase

SGTL5000
  [X] select audio input path *1 (untested)
  [X] set output gain *1 (untested)
  [X] set mic bias and gain *1 (untested)
  [X] configure biquads

AUDIO
  [X] internal 48kHz processing
  [X] clock drift tracker
  [X] mix between input/synth/bluetooth
  [X] auto switch line-in/bluetooth
  [X] high-quality polyphase resampler for clock-drift
  [X] dithering with noise shaping
  [X] play sounds (mp3)
  [X] play log-sweep
  [X] able to do real-time audio streaming to co-processor
  [X] aliasing/imaging verification for resampler
  [X] amp and codec volume changes dB matching
  [X] match line passhthrough gain
  [X] match left/right (especially line-in/out)
  
I2C
  [X] threading-safe
  [X] bus lock detect & clear routine

COMM
  [X] send PCM real-time
  [X] send AVRCP media session data
  [X] send A2DP device name, (dis)connects

SYSTEM
  [X] button and volume processing
  [X] avrcp forwarding
  [X] sound effects with volume overrides (Resources)
  [X] live DSP updates (Vibe)
  [X] over-the-air firmware updates (OTA)

*/

// External controls
#define GPIO_D5 14
#define GPIO_D4 27
#define GPIO_D3 26
#define GPIO_D2 25
#define GPIO_D1 33
#define GPIO_A1 36  // VP

// Define a fixed amplifier volume for sound effects
#define TAS5828M_SFX_GAIN -18 // dB
#define SGTL5000_SFX_GAIN -12 // dB

// Define gain for TAS5828M vs SGTL5000
// To ensure gain matching the gain range of SGTL5000 (it's the more limited one) is used for both drivers
// But TAS5828M may need "overdrive" for low amplitude sources, so this sets an overall relative boost
#define TAS5828M_GAIN_BOOST 0 // dB

// Define ADC calibrated millivolt range
// Note: The ESP32 ADC is wonky and these values may need adjusting between individuals
//       to ensure that the volume input can be scaled to cover the full 0-100% range.
#define ADC_MIN_MV  250
#define ADC_MAX_MV 3100

// I²C addresses
#define I2C_TAS5828M 0x60
#define I2C_SGTL5000 0x0A

// Our system states
typedef enum {
  PLAY_LOGO,
  WAIT_FOR_LOGO,
  IDLE,
  SOURCE_SWITCH_FADEOUT,
  SOURCE_SWITCH_SOUND,
  SOURCE_SWITCH_FADEIN,
  SYNTH_WAIT,
} speaker_state;

// Custom audio events
typedef enum {
  CMD_NOP = EVT_CUSTOM,
  CMD_AUTO_RESUME,
  CMD_AUTO_SUSPEND,
} custom_events;

// Codec and amplifier drivers
#if USE_TAS5828M
TAS5828M_Driver TAS5828M(I2C_TAS5828M);
#endif
SGTL5000_Driver SGTL5000(I2C_SGTL5000);

// Print events to debug serial
static constexpr bool debug_events = true;

// Bluetooth playback state
static bool playing = false;
static bool bluetooth_playing = false;

// The audio input source that should be used
static audio_source source;

// User input
static const bool buttons_simple = false; // Use simple buttons (press+release in one command, no holding)
static audio_command play_pause;
static const int button_pins[] = { GPIO_D1, GPIO_D2, GPIO_D3, GPIO_D4, GPIO_D5 };
static bool buttons[5];

// Current speaker state
static speaker_state state = PLAY_LOGO;

// Automation
static bool volatile automate = true;
static bool volatile block_volume_change = false;

// Volume input filtering
#if HAVE_VOL_POT
static biquad_filter adc_bq_filter;
static biquad_history adc_bq_history;
static float adc_hys_history = 0;
#if USE_TAS5828M
static float tas5828m_gain = SGTL5000_GAIN_LO; // very quiet
#endif
static float sgtl5000_gain = SGTL5000_GAIN_LO; // mute
#else
#if USE_TAS5828M
static const float tas5828m_gain = 0; // gain used when no volume input is available
#endif
static const float sgtl5000_gain = 0; // gain used when no volume input is available
#endif

// Override (or restore) volume for internal playpack
static void override_volume(bool override) {
  if(override) {
    block_volume_change = true;
#if USE_TAS5828M
    TAS5828M.volume(TAS5828M_SFX_GAIN);
#endif
    SGTL5000.volume(SGTL5000_SFX_GAIN);
  } else {
#if USE_TAS5828M
    TAS5828M.volume(tas5828m_gain);
#endif
    SGTL5000.volume(sgtl5000_gain);
    block_volume_change = false;
  }
}

// Handles audio events and defines system behavior
static void audio_event_handler(audio_event e) {

  // Print for debugging
  if(debug_events) {
    switch(e) {
      case EVT_READY: Comm.printf("Audio => Ready\n"); break;
      case EVT_BT_START: Comm.printf("AD2P => Start\n"); break;
      case EVT_BT_STOP: Comm.printf("AD2P => Stop\n"); break;
      case EVT_BT_STABLE: Comm.printf("AD2P => Stable\n"); break;
      case EVT_SYNTH_DONE: Comm.printf("Synth => Done\n"); break;
      case EVT_FADE_DONE: Comm.printf("Fade => Done\n"); break;
      default: Comm.printf("Unknown/custom => %u\n", e); break;
    }
  }

  // Manage bluetooth state
  if(e == EVT_BT_START) {
    bluetooth_playing = true;
    playing = true;
  } else if(e == EVT_BT_STOP) {
    bluetooth_playing = false;
    playing = false;
  } else if(e == EVT_BT_STABLE) {
    source = bluetooth_playing ? SRC_BT : SRC_ANALOG;
  } else if(e == (audio_event)CMD_AUTO_SUSPEND) {
    automate = false;
  } else if(e == (audio_event)CMD_AUTO_RESUME) {
    if(automate == false) {
      state = IDLE;
      Audio.startFade(1.0);
      automate = true;
    }
  }

  if(automate) {
    
    // Manage speaker state
    speaker_state state_before = state;
    switch(state) {
      case PLAY_LOGO:
        source = Audio.getSource();
        override_volume(true);
        Audio.startSynthMP3(RES_MP3_LOGO);
        state = WAIT_FOR_LOGO;
        break;
      case WAIT_FOR_LOGO:
        if (e == EVT_SYNTH_DONE) {
          override_volume(false);
          Audio.startFade(1.0);
          state = IDLE;
        }
        break;
      case IDLE:
        if (source != Audio.getSource()) {
          Audio.startFade(0.0);
          state = SOURCE_SWITCH_FADEOUT;
        }
        break;
      case SOURCE_SWITCH_FADEOUT:
        if (e == EVT_FADE_DONE) {
          Audio.setSource(source);
          override_volume(true);
          Audio.startSynthMP3((source == SRC_BT) ? RES_MP3_CONNECT : RES_MP3_DISCONNECT);
          state = SOURCE_SWITCH_SOUND;
          if (debug_events) Comm.printf("Source => %s\n", source == SRC_ANALOG ? "Analog" : "Bluetooth");
        } else if (source == Audio.getSource()) {
          Audio.startFade(1.0);
          state = SOURCE_SWITCH_FADEIN;
        }
        break;
      case SOURCE_SWITCH_SOUND:
        if (e == EVT_SYNTH_DONE) {
          override_volume(false);
          Audio.startFade(1.0);
          state = SOURCE_SWITCH_FADEIN;
        } else if(source != Audio.getSource()) {
          Audio.setSource(source);
          Audio.startSynthMP3((source == SRC_BT) ? RES_MP3_CONNECT : RES_MP3_DISCONNECT);
          if (debug_events) Comm.printf("Source => %s\n", source == SRC_ANALOG ? "Analog" : "Bluetooth");
        }
        break;
      case SOURCE_SWITCH_FADEIN:
        if (e == EVT_FADE_DONE) {
          state = IDLE;
        } else if (source != Audio.getSource()) {
          Audio.startFade(0);
          state = SOURCE_SWITCH_FADEOUT;
        }
        break;
      case SYNTH_WAIT:
        if (e == EVT_SYNTH_DONE) {
          state = IDLE;
        }
        break;
    }

    // Debug state switching
    if(debug_events && state != state_before) {
      switch(state) {
        case WAIT_FOR_LOGO: Comm.printf("Audio => Wait for Logo\n"); break;
        case IDLE: Comm.printf("Audio => Idle\n"); break;
        case SOURCE_SWITCH_FADEOUT: Comm.printf("Audio => Switch Fadeout\n"); break;
        case SOURCE_SWITCH_SOUND: Comm.printf("Audio => Switch Sound\n"); break;
        case SOURCE_SWITCH_FADEIN: Comm.printf("Audio => Switch Fadein\n"); break;
        default: break; // don't print any others
      }
    }

  }

}

// Handles application-specific commands from co-processor
void command_handler(Command &command) {
  switch(command.readU8()) {
    case 'A': { // Suspend or resume automation
      switch(command.readU8()) {
        case 'S': {
          Audio.postEvent(CMD_AUTO_SUSPEND);
        } break;
        case 'R': {
           Audio.postEvent(CMD_AUTO_RESUME);
        } break;
      }
    } break;
    case 'L': { // Initiate log-sweep
      sweep_data sweep_command = (sweep_data){
        command.readS32(),
        command.readU8(),
        command.readS32(),
        command.readS32(),
        command.readU32()
      };
      Audio.startSynthSweep(sweep_command);
    } break;
    case 'V': { // Volume control
      Audio.startFade(command.readU8() / 255.0);
    } break;
    case 'S': { // Set source
      switch(command.readU8()) {
        case 'A': {
          Audio.setSource(SRC_ANALOG);
        } break;
        case 'B': {
          Audio.setSource(SRC_BT);
        } break;
      }
    } break;
  }
}

#if USE_TAS5828M
void setup_tas5828m() {

  // Note: Hybrid modulation should be used only when a boost supply is tracking the boost PWM signal.
  //       When digital + analog gain + volume are 0 + no filters, a full scale input will yield chipVmax (see below)29.5Vp.
  //       PVDD voltage from boost supply should linearly match the PWM output duty where 0%=0V and 30%=30V.
  //       Upper and lower limits must be maintained depending on exact chip and other system limits.
  TAS5828M.init(SR_48K, TAS5828M_STEREO, TAS5828M_HYBRID);
  TAS5828M.hybridSetup(); // Enable Class-H PWM output

  // Configure filters (DC+sub blocker)
  biquad_filter f;
  TAS5828M.biquadSet(CH_BOTH, 0, f = biquadMake(FILTER_HIGHPASS, 48000, 40, 0.707, 0));

  // Allow hybrid mode buffers to fill
  delay(5);

  // Set final gain stage for hybrid modulation
  constexpr float chipVmax = 33.1f; // TAS2858M: 29.4V TAS5830: 33.1V
  constexpr float psuVmax  = 28.5f; // Measured Vrms/min @ 100% Class-H PWM
  float gainDb = 20.0f * log10f(psuVmax / chipVmax); // ideal gain
  TAS5828M.analogGain(floorf(gainDb * 2.0f) / 2.0f); // round down

  // Enable DAC output (ramps up)
  TAS5828M.gain(0);

}
#endif


// Set up the system
void setup() {

  pinMode(GPIO_A1, INPUT);  // ADC
  pinMode(GPIO_D1, INPUT_PULLUP);
  pinMode(GPIO_D2, INPUT_PULLUP);
  pinMode(GPIO_D3, INPUT_PULLUP);
  pinMode(GPIO_D4, INPUT_PULLUP);
  pinMode(GPIO_D5, INPUT_PULLUP);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // I2C communications with SGTL5000 and TAS5828M
  I2C.init();

  // Communications with co-processor and debug-link
  Comm.setCommandHandler(command_handler);
  Comm.init();

  // Set up Vibe
  Vibe.setBiquadTarget(TAS5828M);
  Audio.setMetaHandler(Vibe.getMetaHandler());

  // Set up OTA firmware updates
#ifdef OTA_KEY
  OTA.init();
  OTA.setMetaHandler(Comm.getMetaHandler());
  Vibe.setMetaHandler(OTA.getMetaHandler());
#else
  Vibe.setMetaHandler(Comm.getMetaHandler());
#endif

  // Audio system
  Audio.setConnectionHandler(Comm.getConnectionHandler());
  Audio.setAudioHandler(Comm.getAudioHandler());
  Audio.setEventHandler(audio_event_handler);
  Audio.init(DEVICE_NAME, SR_48K, false);

  // Audio hardware
#if USE_TAS5828M
  setup_tas5828m();
#endif
  SGTL5000.init(SR_48K);

  // Configure STGL5000
  SGTL5000.lineOutLevel(0);                                 // Enable line output

  // Kick everything off
  Audio.begin();

#if HAVE_VOL_POT
  // Configure ADC filtering
  adc_bq_filter = biquadMake(FILTER_LOWPASS, 100.0, 2.0, 0.7);
#endif  

}

// Map and saturate, like Arduino map() - but for float and with saturation
float satmapf(float value, float x0, float x1, float y0, float y1) {
  value = ((value - x0) / (x1 - x0)) * (y1 - y0) + y0;
  return (value < y0) ? y0 : ((value > y1) ? y1 : value);
}

// Apply hysteresis and scale to account for range loss
float hysteresis(float value, float &memory, float x0, float x1, float hysteresis) {
  if(value - hysteresis > memory) memory = value - hysteresis;
  if(value + hysteresis < memory) memory = value + hysteresis;
  return satmapf(memory, x0 + hysteresis, x1 - hysteresis, x0, x1);
}

#if USE_TAS5828M
// Monitor TAS5828M state and recover from any fault that prevent playback
// This used to be PVDD-overvoltage-specific, but now it is a generic recoverer
// Faults should auto-recover but real overvoltage tests show they generally don't
// While the OV error could be cleared, the PLL was unable to re-aquire a lock
static void monitor_tas5828m() {
  static uint16_t counter = 0;
  if(counter && !--counter) {
    Comm.printf("Resetting TAS...\n");
    setup_tas5828m();
  }
  if(TAS5828M.state() == TAS5828M_PLAY) {
    counter = 0;
  } else if(!counter) {
    tas5828m_status_t status = TAS5828M.status();
    counter = status.overtemp_shutdown ? 1000 : 250; // 10s at overtemp else 2.5s
  }
}
#endif

// Processes user input
void loop() {

#if HAVE_VOL_POT
  // Read volume from ADC as 0.0-1.0
  float volume = satmapf(analogReadMilliVolts(GPIO_A1), ADC_MIN_MV, ADC_MAX_MV, 0.0, 1.0);
  // Apply low-pass filter (this removes most ADC noise and smooths the response)
  volume = biquadProcess(adc_bq_history, adc_bq_filter, volume);
  // Limit precision (prevents chasing tiny fractions)
  volume = round(volume * 1000.0) / 1000.0;
  // Apply hysteresis (this perfectly stabilizes volume input)
  volume = hysteresis(volume, adc_hys_history, 0.0, 1.0, 0.01);

  // Update gains when volume input changes (unless blocked)
  if(!block_volume_change) {
    float chip_gain;
#if USE_TAS5828M
    // Update TAS5828M gain
    chip_gain = satmapf(volume, 0.0, 1.0, SGTL5000_GAIN_LO, SGTL5000_GAIN_HI) + TAS5828M_GAIN_BOOST;
    if(chip_gain != tas5828m_gain) {
      tas5828m_gain = chip_gain;
      TAS5828M.volume(tas5828m_gain);
    }
#endif    
    // Update SGTL5000 gain
    chip_gain = satmapf(volume, 0.0, 1.0, SGTL5000_GAIN_LO, SGTL5000_GAIN_HI);
    if(chip_gain != sgtl5000_gain) {
      sgtl5000_gain = chip_gain;
      SGTL5000.volume(sgtl5000_gain);
    }
  }
#endif

  // Handle buttons
  for(int button = 0; button < 5; button++) {
    if(!digitalRead(button_pins[button]) != buttons[button]) {
      printf("%u %u\n", button, buttons[button]);
      buttons[button] = !buttons[button];
      static audio_command command = CMD_NONE;
      switch(button) {
        case 0:  // play/pause
          if(buttons[button] == HIGH) {
            play_pause = playing ? CMD_PAUSE : CMD_PLAY;
            playing = !playing;
          }
          command = play_pause;
          break;
        case 1:  // prev
          command = CMD_PREV;
          break;
        case 2:  // next
          command = CMD_NEXT;
          break;
        case 3:  // disconnect current user
          if(buttons[button] == HIGH) Audio.disconnectBluetooth();
          break;
        case 4:  // TBD
          break;
      }
      if(command != CMD_NONE) {
        if(buttons[button] == HIGH) {
          if(buttons_simple) Audio.sendCommand(command);
          else Audio.sendCommand(command, true);
        } else if(buttons[button] == LOW && !buttons_simple) {
          Audio.sendCommand(command, false);
        }
      }
    }
  }

#if USE_TAS5828M
  monitor_tas5828m();
#endif  
  
  delay(10); // Run loop at approx 100Hz (needed to debounce buttons and pace ADC filtering, TAS5828M monitoring)

}
