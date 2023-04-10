#include "encoder.hpp"
#include "interfaces.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"
#include "quantizer.hpp"
#include "voltages.hpp"
#include "Timer.hpp"

// #include <functional>
#include <Arduino.h>
#include <cutemodules.h>

// Use different hardware timer interface depending on whether we're running on atmega or rp2040
#if ( defined(ARDUINO_NANO_RP2040_CONNECT) || defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_ADAFRUIT_FEATHER_RP2040) || \
      defined(ARDUINO_GENERIC_RP2040) ) && defined(ARDUINO_ARCH_MBED)
  #define USING_MBED_RPI_PICO		true
#else
  #error "Somehow these defines aren't in place"
  #define USING_MBED_RPI_PICO		false
#endif

#if USING_MBED_RPI_PICO
#define FORCE_REFORMAT false
#include "NonVolatileStorage.hpp"
#endif

#include "pins.hpp"

#define NUM_TIMERS (3)
#define MAX_VOLTAGE (1023)
#define EOM_BUFFER_MICROS (10000)
#define EOM_LED_BUFFER_MICROS (250000)
#define MOD_BUTTON_FLASH_TIME (75000)
#define MOD_BUTTON_FLASH_COUNT \
  (4) // To make this a bit easier to code, this should always be 2x the number
      // of flashes you want
#define MOD_BUTTON_STROBE_SLOW (250000)
#define MOD_BUTTON_RESET_TIME_MICROS (2000000)
#define TEMPO_DISPLAY_TIME (2000000)
#define OTHER_DISPLAY_TIME (2000000)
#define DIV_BUTTON_HOLD_TIME (2000000)
#define BEAT_BUTTON_HOLD_TIME (2000000)
#define COUNTDOWN_DISPLAY_TIME (500000)
#define LATCH_PULSE_TIME (500000)
#define STATE_COMPARE_INTERVAL (100000)
#define STATE_COMMIT_INTERVAL (15000000)
#define PRESET_DISPLAY_TIME (10000000)

// #define IS_POWERED_FROM_ARDUINO

class Module : public ModuleInterface<messd_ins_t, messd_outs_t> {
private:

  static const int beatsDivMin = 2;
  static const int beatsDivMax = 32;
  static const int tempoMin = 10;
  static const int tempoMax = 500;
  static const int tapTempoMin = 40;
  static const int tapTempoMax = 280;
  static const int inputClockDivideMin = 1;
  static const int inputClockDivideMax = 9;
  class MessdUpHardware : public Hardware<MessdUpHardware> {
  private:
    uint8_t muxPins[3] = {MUX_S0, MUX_S1, MUX_S2};

  public:
    // Modulation switch gets its own dedicated pin
    Pin<unsigned char> MODSWITCH = {0, MOD_SW, INPUT};

    // Shift register hardware (seven segment)
    ShiftRegister sevenSegmentOuts = ShiftRegister(SSEG_DS, SSEG_STCP, SSEG_SHCP);
    ShiftRegister moduleOuts = ShiftRegister(OUTS_DS, OUTS_STCP, OUTS_SHCP);
    ShiftRegister moduleLEDs = ShiftRegister(LEDS_DS, LEDS_STCP, LEDS_SHCP);

    SevenSegmentDisplay sevenSegmentDisplay =
        SevenSegmentDisplay(&sevenSegmentOuts, SHIFT_PERMUTATION);

    Mux analogMux = Mux(muxPins, MUX_1_COM, true);
    Mux digitalMux = Mux(muxPins, MUX_2_COM, false);

    Encoder div = Encoder(HIGH, HIGH, 0);
    Encoder beat = Encoder(HIGH, HIGH, 0);
  };

  messd_t messd;
  messd_ins_t ins;
  messd_outs_t outs;

  typedef struct SerializableState {
    int beats;
    int subdivisions;
    float tapTempo;
    uint8_t div_latch;
    uint8_t beat_latch;
    uint8_t beatInputResetMode;
    uint8_t inputClockDivider;
  } SerializableState;

  typedef struct Calibration {
    uint16_t modInputMid;
    uint16_t beatInputMid;
    uint16_t divInputMid;
    uint16_t truncInputMid;
  } Calibration;

  Quantizer _divCVQuantizer;
  Quantizer _beatCVQuantizer;

  void HardwareRead(messd_ins_t *ins, messd_outs_t *outs);
  void HardwareWrite(messd_ins_t *ins, messd_outs_t *outs);

  unsigned long lastProcessTime = 0;
  unsigned long lastRecordedHighClockTime = 0;
  unsigned long measuredPeriod = 500000;
  uint8_t hasProcessedHighClock = false;

  uint8_t digitCounter = 0;

  // Storage for the encoders
  uint8_t divEncA = 0;
  uint8_t divEncB = 0;
  uint8_t encStateDiv = 0;
  uint8_t beatsEncA = 0;
  uint8_t beatsEncB = 0;
  uint8_t encStateBeats = 0;

  // Different module states
  enum class ModuleState {
    Default = 0,
    Tempo,
    Preset,
    ParamMenu
  };

  ModuleState _currentState = ModuleState::Default;

  // Different display types
  enum class DisplayState {
    Default = 0,
    Tempo,
    Pop,
    InputClockDivide,
    BeatMode,
    Countdown,
    BeatsEqualDivs,
    Preset,
    Calibration,
    Done
  };

  DisplayState displayState = DisplayState::Default;

  // Storage for the clock switch
  uint8_t clockSwitch = LOW;
  uint8_t isClockInternal = 1;
  float scaledTempo = 120.0f;
  unsigned long lastTapMicros = 0;
  unsigned char totalTaps = 0;
  uint32_t tempoDisplayTime = TEMPO_DISPLAY_TIME;
  uint8_t isRoundTripMode = 1;

  // Beats and subdivisions
  int activeBeats;
  int activeDiv;

  // Storage for the modulation switch
  uint8_t modSwitch = HIGH; // active low
  uint8_t canTriggerReset = 1;
  float eomBuffer = 0.0f;
  float modHoldTime = 0.0;
  uint8_t modButtonFlashCount = MOD_BUTTON_FLASH_COUNT;
  float modButtonFlashTimer = 0.0f;
  uint8_t modulationButtonIgnored = 0;
  Timer _beatsEqualsDivTimer;

  // Holding down the div encoder switch
  float divHoldTime = 0.0;
  uint32_t inputClockDivDisplayTime = OTHER_DISPLAY_TIME;

  // Holding down the beat encoder switch
  float beatHoldTime = 0.0;
  uint32_t beatModeDisplayTime = OTHER_DISPLAY_TIME;
  uint8_t lastBeatInputValue = 0;
  t_thresh beatSwitchThreshold;

  // Countdown display
  uint8_t lastDownbeat = false;
  uint32_t countdownDisplayTime = COUNTDOWN_DISPLAY_TIME;
  uint16_t countdownSampleAndHold = 0;

  // Calibration display and state
  uint8_t doCalibrate = false;
  uint8_t calibrationPossible = true;
  uint32_t calibrateDisplayTime = OTHER_DISPLAY_TIME;
  Calibration calibratedState = {
    MOD_INPUT_MID, BEAT_INPUT_MID, DIV_INPUT_MID, TRUNC_INPUT_MID
  };

  // Storage for animations on the modulate button
  bool animateModulateButton = false;
  float animateModulateButtonTime = 0.0f;

  // Storage for output shift register
  uint8_t output_sr_val[8];
  enum class OutputNames {
    Nothing = 0,
    TruncateLED,
    DivLED,
    EoMOutput,
    TruncateOutput,
    DivOutput,
    BeatOutput,
    DownbeatOutput
  };

  // Storage for LED shift register
  uint8_t leds_sr_val[8];
  enum class LEDNames {
    ModLEDButton = 0,
    EoMLED,
    ClockLEDButton,
    BeatLED,
    DownbeatLED,
    BeatLatchLED,
    DivLatchLED,
    Nothing
  };

  // Storage for the latch switches
  uint8_t initial_div_latch = 0;
  uint8_t initial_beat_latch = 0;
  uint8_t beat_switch_state_prev = 0;
  uint8_t div_switch_state_prev = 0;
  uint8_t canSwitchBeatInputModes = 1;
  bool _beatLatchFlashState = false;
  Timer _beatLatchFlashTimer;
  bool _divLatchFlashState = false;
  Timer _divLatchFlashTimer;

  // Serializable state that can go into EEPROM
  SerializableState activeState = {
    4,        // beats
    7,        // subdivisions
    131.0,    // tempo
    1,        // div_latch
    1,        // beat_latch
    0,        // beatInputResetMode
    1         // inputClockDivider
  };
  enum class PresetAction {
    None = 0,
    Store,
    Recall
  };
  uint32_t stateCompareTimer = STATE_COMPARE_INTERVAL;
  uint32_t stateCommitTimer = STATE_COMMIT_INTERVAL;
  NonVolatileStorage<SerializableState> _nonVolatileStorage;
  uint8_t _nonVolatileStorageInitialized = false;
  uint32_t presetDisplayTimer = PRESET_DISPLAY_TIME;
  uint8_t targetPresetIndex = 0;
  PresetAction presetAction = PresetAction::None;
  uint32_t doneDisplayTimer = OTHER_DISPLAY_TIME;

  Timer *_timers[NUM_TIMERS] = {
    &_beatsEqualsDivTimer,
    &_beatLatchFlashTimer,
    &_divLatchFlashTimer
  };

  void _beatsEqualsDivCallback(float progress);
  void _beatLatchTimerCallback(float progress);
  void _divLatchTimerCallback(float progress);

  void _initializeFromSavedData();
  void _transitionCurrentState();
  void _scaleValues();
  void _processEncoders(float ratio);
  void _processTapTempo(float msDelta);
  void _processModSwitch(float msDelta);
  void _processBeatDivSwitches(float msDelta);
  void _processCalibration(float microsdelta);
  void _display();
  void _displayLatchLEDs();

public:
  Module();
  MessdUpHardware hardware;

  // TODO: move this
  bool shouldDisplayBeatsEqualsDivs = false;

#if (USING_MBED_RPI_PICO)

#if REVISION == 2
  struct AnalogMux {
    int BEAT_INPUT = 0;
    int TRUNCATE_ATV = 1;
    int ROUND_SWITCH = 2;
    int DIVIDE_ATV = 3;
    int LATCH_SWITCH = 4;
    int DIVIDE_INPUT = 5;
    int MOD_INPUT = 6;
    int TRUNCATE_INPUT = 7;
  } AnalogMux;

#elif REVISION == 1

  struct AnalogMux {
    int BEAT_INPUT = 0;
    int TRUNCATE_ATV = 1;
    int ROUND_SWITCH = 2;
    int DIVIDE_ATV = 3;
    int LATCH_SWITCH = 4;
    int DIVIDE_INPUT = 5;
    int MOD_INPUT = 6;
    int TRUNCATE_INPUT = 7;
  } AnalogMux;

#else
#error "Revision undefined"
#endif

#else
  struct AnalogMux {
    int DIVIDE_ATV = 0;
    int TRUNCATE_ATV = 1;
    int ROUND_SWITCH = 2;
    int BEAT_INPUT = 3;
    int LATCH_SWITCH = 4;
    int DIVIDE_INPUT = 5;
    int MOD_INPUT = 6;
    int TRUNCATE_INPUT = 7;
  } AnalogMux;
#endif
  struct DigitalMux {
    int BEAT_ENC_A = 0;
    int BEAT_ENC_B = 1;
    int BEAT_SWITCH = 2;
    int DIV_SWITCH = 3;
    int DIVIDE_ENC_B = 4;
    int CLOCK_IN = 5; // should be nothing...
    int DIVIDE_ENC_A = 6;
    int CLOCK_SWITCH = 7;
  } DigitalMux;

  void initHardware();
  void process(float microsDelta);
};
