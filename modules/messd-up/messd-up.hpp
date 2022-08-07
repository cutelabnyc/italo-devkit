#include "encoder.hpp"
#include "interfaces.hpp"
#include "mux.hpp"
#include "seven-segment.hpp"

#include <Arduino.h>
#include <cutemodules.h>

#define MAX_VOLTAGE (1023)
#define EOM_BUFFER_MS (10)
#define EOM_LED_BUFFER_MS (250)
#define MOD_BUTTON_FLASH_TIME (75)
#define MOD_BUTTON_FLASH_COUNT                                                 \
  (4) // To make this a bit easier to code, this should always be 2x the number
      // of flashes you want
#define MOD_BUTTON_STROBE_SLOW (250)
#define MOD_BUTTON_RESET_TIME_MS (2000)
#define TEMPO_DISPLAY_TIME (2000)
#define OTHER_DISPLAY_TIME (2000)
#define DIV_BUTTON_HOLD_TIME (2000)
#define BEAT_BUTTON_HOLD_TIME (2000)
#define COUNTDOWN_DISPLAY_TIME (500)
#define LATCH_PULSE_TIME (500)

// #define IS_POWERED_FROM_ARDUINO

class Module : public ModuleInterface<messd_ins_t, messd_outs_t> {
private:
  messd_t messd;
  messd_ins_t ins;
  messd_outs_t outs;

  class MessdUpHardware : public Hardware<MessdUpHardware> {
  private:
    // TODO: Put the actual pin addresses in their own header file so
    // that we can define variations controlled by compiler flags to
    // account for different chips
    uint8_t muxPins[3] = {4, 3, 2};

  public:
    // Modulation switch gets its own dedicated pin
    Pin<unsigned char> MODSWITCH = {0, A3, INPUT};

    // Shift register hardware (seven segment)
    ShiftRegister sevenSegmentOuts = ShiftRegister(11, 9, 10);
    ShiftRegister moduleOuts = ShiftRegister(7, 6, 5);
    ShiftRegister moduleLEDs = ShiftRegister(8, A4, A5);

    SevenSegmentDisplay sevenSegmentDisplay =
        SevenSegmentDisplay(&sevenSegmentOuts);

    Mux analogMux = Mux(muxPins, A0, true);
    Mux digitalMux = Mux(muxPins, A2, false);

    Encoder div = Encoder(HIGH, HIGH, 0);
    Encoder beat = Encoder(HIGH, HIGH, 0);
  };

  void HardwareRead(messd_ins_t *ins, messd_outs_t *outs);
  void HardwareWrite(messd_ins_t *ins, messd_outs_t *outs);

  unsigned long lastProcessTime = 0;
  unsigned long lastRecordedHighClockTime = 0;
  unsigned long measuredPeriod = 500000;
  uint8_t hasProcessedHighClock = false;

  // consts
  static const int beatsDivMin = 2;
  static const int beatsDivMax = 32;
  static const int tempoMin = 10;
  static const int tempoMax = 500;
  static const int tapTempoMin = 40;
  static const int tapTempoMax = 280;
  static const int inputClockDivideMin = 1;
  static const int inputClockDivideMax = 9;

  uint8_t digitCounter = 0;

  // Storage for the encoders
  uint8_t divEncA = 0;
  uint8_t divEncB = 0;
  uint8_t encStateDiv = 0;
  uint8_t beatsEncA = 0;
  uint8_t beatsEncB = 0;
  uint8_t encStateBeats = 0;

  // Different display types
  enum class DisplayState {
    Default = 0,
    Tempo,
    Pop,
    InputClockDivide,
    BeatMode,
    Countdown,
    BeatsEqualDivs
  };

  DisplayState displayState = DisplayState::Default;

  // Storage for the clock switch
  uint8_t clockSwitch = LOW;
  float tapTempo = 131.0;
  float scaledTempo = 120.0f;
  unsigned long lastTapMicros = 0;
  unsigned char totalTaps = 0;
  uint16_t tempoDisplayTime = TEMPO_DISPLAY_TIME;

  // Storage for the modulation switch
  uint8_t modSwitch = HIGH; // active low
  uint8_t canTriggerReset = 1;
  float eomBuffer = 0.0f;
  float modHoldTime = 0.0;
  uint8_t modButtonFlashCount = MOD_BUTTON_FLASH_COUNT;
  float modButtonFlashTimer = 0.0f;
  uint8_t modulationButtonIgnored = 0;
  uint16_t beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME;

  // Holding down the div encoder switch
  float divHoldTime = 0.0;
  uint16_t inputClockDivDisplayTime = OTHER_DISPLAY_TIME;

  // Holding down the beat encoder switch
  float beatHoldTime = 0.0;
  uint16_t beatModeDisplayTime = OTHER_DISPLAY_TIME;
  uint8_t lastBeatInputValue = 0;

  // Countdown display
  uint8_t lastDownbeat = false;
  uint16_t countdownDisplayTime = COUNTDOWN_DISPLAY_TIME;
  uint16_t countdownSampleAndHold = 0;

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
    DownbeatOutput,
    BeatOutput
  };

  // Storage for LED shift register
  uint8_t leds_sr_val[8];
  enum class LEDNames {
    ModLEDButton = 0,
    EoMLED,
    ClockLEDButton,
    DownbeatLED,
    BeatLED,
    BeatLatchLED,
    DivLatchLED,
    Nothing
  };

  // Storage for the latch switches
  uint8_t beat_latch = 1;
  uint8_t div_latch = 1;
  uint8_t initial_div_latch = 0;
  uint8_t initial_beat_latch = 0;
  uint8_t beat_switch_state_prev = 0;
  uint8_t div_switch_state_prev = 0;
  uint8_t canSwtichBeatInputModes = 1;
  float latchPulseTimer = 0.0f;

  void _scaleValues();
  void _processEncoders();
  void _processTapTempo(float msDelta);
  void _processModSwitch(float msDelta);
  void _processBeatDivSwitches(float msDelta);
  void _display();

public:
  Module();
  MessdUpHardware hardware;

  struct state {
    int beats = 4;
    int div = 7;
    int activeBeats = 4;
    int activeDiv = 7;
  } state;

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

  struct DigitalMux {
    int BEAT_ENC_A = 0;
    int BEAT_ENC_B = 1;
    int BEAT_SWITCH = 2;
    int DIV_SWITCH = 3;
    int DIVIDE_ENC_B = 4;
    int CLOCK_IN = 5;
    int DIVIDE_ENC_A = 6;
    int CLOCK_SWITCH = 7;
  } DigitalMux;

  void initHardware();
  void process(float microsDelta);
};
