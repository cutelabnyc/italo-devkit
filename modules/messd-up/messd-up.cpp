#include "messd-up.hpp"
#include "timers.hpp"

#include <avr/interrupt.h>
#include <functional>
using namespace std::placeholders;

#if USING_MBED_RPI_PICO
  #include "MBED_RPi_Pico_TimerInterrupt.h"

  MBED_RPI_PICO_Timer ITimer0(0);
#else
  #define USE_TIMER_1 true
  #define USE_TIMER_2 true
  #include <TimerInterrupt.h>
#endif

int clockIn = CLK_IN_A;
int clockJackSwitch = CLK_JACK_SWITCH;

// More accurate input clock high detection
volatile uint8_t lastInternalClockState = LOW;
volatile unsigned long lastHighClockTime = 0;
uint16_t clockPhaseCounter = 0;
volatile uint8_t inputClockCounter = 0;
volatile uint8_t atomicInputClockDivider = 1;
volatile uint8_t lastExternalClock = LOW;

// Tempo
volatile float tapTempoOut = 131.0;
float lastTapTempoOut = 131.0;

#define DIV_INPUT_CALIBRATION (-9)

static TIMER_HEADER(TimerHandler)
  clockPhaseCounter++;

  if (clockPhaseCounter >= 100) {
    clockPhaseCounter = 0;
    lastInternalClockState = !lastInternalClockState;
  }

  if (lastTapTempoOut != tapTempoOut) {
    float timerFreqHz = tapTempoOut * 100.0f /
                        30.0f; // double speed because it alternates at 2z Hz
    lastTapTempoOut = tapTempoOut;
#if USING_MBED_RPI_PICO
    ITimer0.setFrequency(timerFreqHz, TimerHandler);
#else
    ITimer1.setFrequency(timerFreqHz, TimerHandler);
#endif
  }
TIMER_FOOTER()

#if USING_MBED_RPI_PICO
void clockPinInterrupt() {
  if (digitalRead(clockIn)) {
#else
ISR(PCINT0_vect) {
  if (PINB & 0b00010000) {
#endif
    if (inputClockCounter == 0) {
      lastHighClockTime = micros();
      lastExternalClock = HIGH;
    } else {
      lastExternalClock = LOW;
    }

    inputClockCounter = (inputClockCounter + 1) % atomicInputClockDivider;
  } else {
    lastExternalClock = LOW;
  }
}

#pragma mark Timer Callbacks
void Module::_clearTemporaryDisplayCallback(float progress)
{
  if (progress >= 1.0f) {
    _temporaryDisplayState = Module::TemporaryDisplayState::None;
  }
}

void Module::_beatLatchTimerCallback(float progress)
{
  _beatLatchFlashState = (progress < 0.5f);
}

void Module::_divLatchTimerCallback(float progress)
{
  _divLatchFlashState = (progress < 0.5f);
}

void Module::_presetTimerCallback(float progress)
{
  if (progress >= 1.0f) {
    if (_currentState == ModuleState::Preset) {
      _currentState = ModuleState::Default;
    }
  }
}

void Module::_paramMenuTimerCallback(float progress)
{
  if (progress >= 1.0f) {
    if (
      _currentState == ModuleState::ParamMenu
      || _currentState == ModuleState::BeatInput
      || _currentState == ModuleState::BeatMult
      || _currentState == ModuleState::DivMult
      || _currentState == ModuleState::ClockCount
      || _currentState == ModuleState::Duty
      || _currentState == ModuleState::ModStyle
      || _currentState == ModuleState::ClockStop
    ) {
      _currentState = ModuleState::Default;
    }
  }
}

void Module::_divButtonTimerCallback(float progress)
{
  if (progress >= 1.0f) {
    // Enter calibration if both buttons have been held down since the start
    if (hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH) == LOW && calibrationPossible) {
      doCalibrate = true;
      Serial.println("Calibrate");
      _displayTemporaryWithTimer(TemporaryDisplayState::Calibrate, &_calibrateDisplayTimer, OTHER_DISPLAY_TIME);
      activeState.beat_latch = initial_beat_latch;
      activeState.div_latch = initial_div_latch;
    } else {
      if (_currentState == ModuleState::ParamMenu
        || _currentState == ModuleState::BeatInput
        || _currentState == ModuleState::BeatMult
        || _currentState == ModuleState::DivMult
        || _currentState == ModuleState::ClockCount
        || _currentState == ModuleState::Duty
        || _currentState == ModuleState::ModStyle
        || _currentState == ModuleState::ClockStop
      ) {
        _currentState = ModuleState::Default;
      } else {
        activeState.div_latch = initial_div_latch;
        _currentState = ModuleState::ParamMenu;
        _paramMenuDisplayTimer.start(PRESET_DISPLAY_TIME);
      }
    }
  }
}

void Module::_beatButtonTimerCallback(float progress)
{
  if (progress >= 1.0f) {
    // Enter calibration if both buttons have been held down since the start
    if (hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH) == LOW) {
      if (calibrationPossible) {
        doCalibrate = true;
        Serial.println("Calibrate");
        _displayTemporaryWithTimer(TemporaryDisplayState::Calibrate, &_calibrateDisplayTimer, OTHER_DISPLAY_TIME);
        activeState.beat_latch = initial_beat_latch;
        activeState.div_latch = initial_div_latch;
      }
    } else {
      activeState.beat_latch = initial_beat_latch;
      _currentState = ModuleState::Preset;
      _presetDisplayTimer.start(PRESET_DISPLAY_TIME);
    }
  }
}

#pragma mark -

void Module::_initializeFromSavedData()
{
  // Load initial preset
  SerializableState initialData;
  uint8_t loaded = _nonVolatileStorage.loadLastPreset(&initialData);
  if (loaded) {
    memcpy(&activeState, &initialData, sizeof(SerializableState));
    tapTempoOut = activeState.tapTempo;
  }

  // Load calibration
  Calibration calibration;
  char buff[128];
  sprintf(buff, "/littlefs/calibration.txt");
  loaded = _nonVolatileStorage.read(buff, &calibration);
  if (loaded) {
    memcpy(&calibratedState, &calibration, sizeof(Calibration));
  }
  _beatCVQuantizer.setInMid(calibratedState.beatInputMid);
  _divCVQuantizer.setInMid(calibratedState.divInputMid);

  _nonVolatileStorageInitialized = true;
}

void Module::_scaleValues() {
  this->ins.tempo = (this->ins.tempo > 0 ? this->ins.tempo : 1);
  this->ins.beatsPerMeasure =
      (this->ins.beatsPerMeasure > 0 ? this->ins.beatsPerMeasure : 1);
  this->ins.subdivisionsPerMeasure =
      (this->ins.subdivisionsPerMeasure > 0 ? this->ins.subdivisionsPerMeasure
                                            : 1);
  this->ins.phase =
      (this->ins.phase <= 1 && ins.phase >= 0 ? this->ins.phase : 0);
  this->ins.pulseWidth = (this->ins.pulseWidth < 1 && this->ins.pulseWidth > 0
                              ? this->ins.pulseWidth
                              : 0.5);
}

void Module::_processEncoders(float ratio) {
  int inc = hardware.beat.process(
      hardware.digitalMux.getOutput(DigitalMux.BEAT_ENC_A),
      hardware.digitalMux.getOutput(DigitalMux.BEAT_ENC_B));

  // If we're using the internal clock,
  // then we want to make changes relative to the scaled clock
  float incScale = ratio;

  if (inc != 0) {
    if (_currentState == ModuleState::Default) {
      // You can't change beats when you're in a round trip modulation, in latch
      // mode
      if (!(messd.inRoundTripModulation && ins.latchModulationToDownbeat)) {
        activeState.beats += inc;
        if (activeState.beats < beatsDivMin)
          activeState.beats = beatsDivMax;
        if (activeState.beats > beatsDivMax)
          activeState.beats = beatsDivMin;
      }
    }

    else if (_currentState == ModuleState::Tempo) {
      activeState.tapTempo += inc * incScale * 0.1;
      tapTempoOut += inc * incScale * 0.1;
      if (activeState.tapTempo < Module::tempoMin)
        activeState.tapTempo = tapTempoOut = Module::tempoMin;
      if (activeState.tapTempo > Module::tempoMax)
        activeState.tapTempo = tapTempoOut = Module::tempoMax;
    }

    else if (_currentState == ModuleState::Preset) {
      _presetDisplayTimer.restart();
      if (presetAction == PresetAction::None || presetAction == PresetAction::Store) {
        presetAction = PresetAction::Recall;
      } else {
        presetAction = PresetAction::Store;
      }
    }

    else if (_currentState == ModuleState::ParamMenu) {
      // Beat encoder is a no-op here
      _paramMenuDisplayTimer.restart();
    }
  }

  inc = hardware.div.process(
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_A),
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_B));

  if (inc != 0) {

    if (_currentState == ModuleState::Default) {
      activeState.subdivisions += inc;
      if (activeState.subdivisions < beatsDivMin)
        activeState.subdivisions = beatsDivMax;
      if (activeState.subdivisions > beatsDivMax)
        activeState.subdivisions = beatsDivMin;
    }

    else if (_currentState == ModuleState::Tempo) {
      activeState.tapTempo += inc * 10 * incScale;
      tapTempoOut += inc * 10 * incScale;
      if (activeState.tapTempo < Module::tempoMin)
        activeState.tapTempo = tapTempoOut = Module::tempoMin;
      if (activeState.tapTempo > Module::tempoMax)
        activeState.tapTempo = tapTempoOut = Module::tempoMax;
    }

    else if (_currentState == ModuleState::Preset) {
      _presetDisplayTimer.restart();
      targetPresetIndex += inc;
      if (targetPresetIndex < 0) targetPresetIndex = 9 - ( abs(targetPresetIndex) % 9);
      if (targetPresetIndex >= 9) targetPresetIndex %= 9;
    }

    else if (_currentState == ModuleState::ParamMenu) {
      _menuIndex += inc;
      if (_menuIndex >= 7) _menuIndex = 0;
      if (_menuIndex < 0) _menuIndex = 6;
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::BeatInput) {
      activeState.beatInputResetMode = !activeState.beatInputResetMode;
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::BeatMult) {
      if (inc >= 1) {
        activeState.beatMult = activeState.beatMult << 1;
        if (activeState.beatMult >= (1 << 4)) {
          activeState.beatMult = 1;
        }
      } else if (inc < 0) {
        if (activeState.beatMult == 1) {
          activeState.beatMult = (1 << 3);
        } else {
          activeState.beatMult = activeState.beatMult >> 1;
        }
      }
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::DivMult) {
      if (inc >= 1) {
        activeState.divMult = activeState.divMult << 1;
        if (activeState.divMult >= (1 << 4)) {
          activeState.divMult = 1;
        }
      } else if (inc < 0) {
        if (activeState.divMult == 1) {
          activeState.divMult = (1 << 3);
        } else {
          activeState.divMult = activeState.divMult >> 1;
        }
      }
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::ClockCount) {
      activeState.inputClockDivider += inc;
      if (activeState.inputClockDivider < Module::inputClockDivideMin) {
        activeState.inputClockDivider = Module::inputClockDivideMin;
      } else if (activeState.inputClockDivider > Module::inputClockDivideMax) {
        activeState.inputClockDivider = Module::inputClockDivideMax;
      }
      atomicInputClockDivider = activeState.inputClockDivider;
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::Duty) {
      activeState.fixedDutyCycle = !activeState.fixedDutyCycle;
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::ModStyle) {
      activeState.modulationStyle += inc;
      if (activeState.modulationStyle < 0) {
        activeState.modulationStyle = 2;
      } else if (activeState.modulationStyle > 2) {
        activeState.modulationStyle = 0;
      }
      _paramMenuDisplayTimer.restart();
    }

    else if (_currentState == ModuleState::ClockStop) {
      activeState.clockStop += inc;
      if (activeState.clockStop < 0) {
        activeState.clockStop = 1;
      } else if (activeState.clockStop > 1) {
        activeState.clockStop = 0;
      }
      _paramMenuDisplayTimer.restart();
    }
  }
}

void Module::_processTapTempo(float microsDelta) {
  int nextClockSwitch = hardware.digitalMux.getOutput(DigitalMux.CLOCK_SWITCH);

  if (isClockInternal) {
    if (nextClockSwitch == LOW && this->clockSwitch == HIGH) {
      unsigned long nextTapMicros = micros();
      unsigned long delta;
      if (nextTapMicros < this->lastTapMicros) {
        delta = (4294967295 - this->lastTapMicros) + nextTapMicros;
      } else {
        delta = nextTapMicros - this->lastTapMicros;
      }

      // Treat it as the first tap if it's been more than two seconds
      if (delta > 2000000 || this->lastTapMicros == 0) {
        totalTaps = 1;
      } else if (totalTaps == 1) {
        activeState.tapTempo = 60000000.0 / ((double)delta);
        totalTaps++;
      } else {
        activeState.tapTempo =
            (60000000.0 / ((double)delta)) * (1.0 / (double)totalTaps) +
            activeState.tapTempo * ((totalTaps - 1) / (double)totalTaps);
        tapTempoOut = activeState.tapTempo;
        totalTaps = totalTaps >= 5 ? 5 : (totalTaps + 1);
      }

      this->lastTapMicros = nextTapMicros;

      if (activeState.tapTempo < Module::tapTempoMin)
        activeState.tapTempo = tapTempoOut = Module::tapTempoMin;
      if (activeState.tapTempo > Module::tapTempoMax)
        activeState.tapTempo = tapTempoOut = Module::tapTempoMax;
    }

  }

  if (nextClockSwitch == LOW) {
    _displayTemporaryWithTimer(TemporaryDisplayState::Tempo, &_tempoDisplayTimer, TEMPO_DISPLAY_TIME);
    _currentState = ModuleState::Tempo;
  } else if (_currentState == ModuleState::Tempo) {
    _currentState = ModuleState::Default;
  }

  this->clockSwitch = nextClockSwitch;
}

void Module::_processModSwitch(float microsDelta) {
  /* hardware.MODSWITCH.PinRead(); */
  hardware.MODSWITCH.val = digitalRead(hardware.MODSWITCH.address);

  if (hardware.MODSWITCH.val == LOW) {
    this->modHoldTime += microsDelta;
  } else {
    this->modHoldTime = 0;
    this->canTriggerReset = true;
  }

  this->modSwitch = hardware.MODSWITCH.val;
}

void Module::_divSwitchPressed() {
  if (_currentState == ModuleState::Default) {
    initial_div_latch = activeState.div_latch;
    activeState.div_latch = !activeState.div_latch;
    _divButtonHoldTimer.start(BEATDIV_BUTTON_HOLD_TIME);
  } else if (_currentState == ModuleState::Preset) {
    presetAction = PresetAction::None;
    _currentState = ModuleState::Default;
    _presetDisplayTimer.clear();
  } else if (_currentState == ModuleState::ParamMenu) {
    if (_menuIndex == 0) {
      _currentState = ModuleState::ClockCount;
    } else if (_menuIndex == 1) {
      _currentState = ModuleState::BeatMult;
    } else if (_menuIndex == 2) {
      _currentState = ModuleState::DivMult;
    } else if (_menuIndex == 3) {
      _currentState = ModuleState::BeatInput;
    } else if (_menuIndex == 4) {
      _currentState = ModuleState::Duty;
    } else if (_menuIndex == 5) {
      _currentState = ModuleState::ModStyle;
    } else if (_menuIndex == 6) {
      _currentState = ModuleState::ClockStop;
    }
    _divButtonHoldTimer.start(BEATDIV_BUTTON_HOLD_TIME);
    _paramMenuDisplayTimer.restart();
  } else if (
    _currentState == ModuleState::ClockCount
    || _currentState == ModuleState::BeatMult
    || _currentState == ModuleState::DivMult
    || _currentState == ModuleState::BeatInput
    || _currentState == ModuleState::Duty
    || _currentState == ModuleState::ModStyle
    || _currentState == ModuleState::ClockStop
  ) {
    _divButtonHoldTimer.start(BEATDIV_BUTTON_HOLD_TIME);
    _currentState = ModuleState::ParamMenu;
    _paramMenuDisplayTimer.restart();
  }
}

void Module::_beatSwitchPressed() {
  if (_currentState == ModuleState::Default) {
    initial_beat_latch = activeState.beat_latch;
    activeState.beat_latch = !activeState.beat_latch;
    _beatButtonHoldTimer.start(BEATDIV_BUTTON_HOLD_TIME);
  } else if (_currentState == ModuleState::Preset) {
    bool displayDone = false;
    if (presetAction == PresetAction::Recall) {
      _nonVolatileStorage.readPreset(targetPresetIndex, &activeState);
      this->ins.reset = true;
      tapTempoOut = activeState.tapTempo;
      displayDone = true;
    } else if (presetAction == PresetAction::Store) {
      _nonVolatileStorage.storePreset(targetPresetIndex, &activeState);
      displayDone = true;
    }

    if (displayDone) {
      _displayTemporaryWithTimer(
        TemporaryDisplayState::Done,
        &_doneDisplayTimer,
        OTHER_DISPLAY_TIME
      );
    }

    presetAction = PresetAction::None;
    _currentState = ModuleState::Default;
    _presetDisplayTimer.clear();
  } else if (_currentState == ModuleState::ParamMenu) {
    // no-op
  } else if (
    _currentState == ModuleState::ClockCount
    || _currentState == ModuleState::BeatMult
    || _currentState == ModuleState::DivMult
    || _currentState == ModuleState::BeatInput
    || _currentState == ModuleState::Duty
    || _currentState == ModuleState::ModStyle
    || _currentState == ModuleState::ClockStop
  ) {
    _currentState = ModuleState::Default;
    _paramMenuDisplayTimer.restart();
  }
}

void Module::_processBeatDivSwitches(float microsDelta) {

  // div switch
  if (hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH) == LOW) {
    if (div_switch_state_prev == HIGH) {
      _divSwitchPressed();
    }
  } else {
    _divButtonHoldTimer.clear();
    calibrationPossible = false;
  }
  div_switch_state_prev =
    hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH);

  // beat switch
  if (hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH) == LOW) {
    if (beat_switch_state_prev == HIGH) {
      _beatSwitchPressed();
    }
  } else {
    _beatButtonHoldTimer.clear();
    calibrationPossible = false;
  }

  beat_switch_state_prev =
      hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH);
}

void Module::_processCalibration(float microsdelta)
{
  if (doCalibrate) {
    calibratedState.modInputMid =  hardware.analogMux.getOutput(AnalogMux.MOD_INPUT);
    calibratedState.divInputMid =  hardware.analogMux.getOutput(AnalogMux.DIVIDE_INPUT);
    calibratedState.beatInputMid = hardware.analogMux.getOutput(AnalogMux.BEAT_INPUT);
    calibratedState.truncInputMid = hardware.analogMux.getOutput(AnalogMux.TRUNCATE_INPUT);

    calibratedState.divInputMid = DIV_INPUT_MAX - calibratedState.divInputMid + DIV_INPUT_MIN; // invert
    calibratedState.beatInputMid = BEAT_INPUT_MAX - calibratedState.beatInputMid + BEAT_INPUT_MIN; // invert
    calibratedState.truncInputMid = TRUNC_INPUT_MAX - calibratedState.truncInputMid + TRUNC_INPUT_MIN; // invert

    char buff[128];
    sprintf(buff, "/littlefs/calibration.txt");
    _nonVolatileStorage.store(buff, &calibratedState);

    doCalibrate = false;
  }
}

// Different display types
enum class DisplayState {
  Default = 0,
  Tempo,
  Pop,
  InputClockDivide,
  BeatMult,
  DivMult,
  BeatMode,
  Countdown,
  BeatsEqualDivs,
  Preset,
  Calibration,
  Done,
  ParamMenu,
  Duty,
  ModStyle,
  ClockStop
};

static int paramMenuNames[7][4] = {
  // Clock Count (CLCT)
  {
    (int) SpecialDigits::C,
    (int) SpecialDigits::L,
    (int) SpecialDigits::C,
    (int) SpecialDigits::T
  },
  // Beat Multiply (BCNT)
  {
    (int) SpecialDigits::B,
    (int) SpecialDigits::C,
    (int) SpecialDigits::N,
    (int) SpecialDigits::T
  },
  // Div Multiply (DCNT)
  {
    (int) SpecialDigits::D,
    (int) SpecialDigits::C,
    (int) SpecialDigits::N,
    (int) SpecialDigits::T
  },
  // Beat Input Mode (BEAT)
  {
    (int) SpecialDigits::B,
    (int) SpecialDigits::E,
    (int) SpecialDigits::A,
    (int) SpecialDigits::T
  },
  // Duty Cycle (DUTY)
  {
    (int) SpecialDigits::D,
    (int) SpecialDigits::U,
    (int) SpecialDigits::T,
    (int) SpecialDigits::Y
  },
  // Modulation Style (STYL)
  {
    5,
    (int) SpecialDigits::T,
    (int) SpecialDigits::Y,
    (int) SpecialDigits::L
  },
  // Clock Stop (CSTP)
  {
    (int) SpecialDigits::C,
    5,
    (int) SpecialDigits::T,
    (int) SpecialDigits::P
  }
};

static int modStyleNames[3][4] = {
  // Sync (SYNC)
  {
    (int) 5,
    (int) SpecialDigits::Y,
    (int) SpecialDigits::N,
    (int) SpecialDigits::C
  },
  // Stay (STAY)
  {
    (int) 5,
    (int) SpecialDigits::T,
    (int) SpecialDigits::A,
    (int) SpecialDigits::Y
  },
  // Flip (FLIP)
  {
    (int) SpecialDigits::F,
    (int) SpecialDigits::L,
    (int) 1,
    (int) SpecialDigits::P
  },
};

void Module::_display() {
  DisplayState displayState = DisplayState::Default;

  digitCounter++;
  if (digitCounter == 4) {
    digitCounter = 0;
  }

  int value, decimal = 0, colon = 0;
  int tempoDecimal = 1;
  float activeTempo = this->scaledTempo;
  long displayableTempo = (long)round(activeTempo * 10.0f);
  if (displayableTempo > 10000) {
    displayableTempo /= 10;
    displayableTempo %= 10000;
    tempoDecimal = 0;
  }

  if (this->outs.resetPending) {
    displayState = DisplayState::Pop;
  } else if (_currentState == ModuleState::Tempo) {
    displayState = DisplayState::Tempo;
  } else if (_currentState == ModuleState::Preset) {
    displayState = DisplayState::Preset;
  } else if (_currentState == ModuleState::ParamMenu) {
    displayState = DisplayState::ParamMenu;
  } else if (_currentState == ModuleState::BeatInput) {
    displayState = DisplayState::BeatMode;
  } else if (_currentState == ModuleState::BeatMult) {
    displayState = DisplayState::BeatMult;
  } else if (_currentState == ModuleState::DivMult) {
    displayState = DisplayState::DivMult;
  } else if (_currentState == ModuleState::ClockCount) {
    displayState = DisplayState::InputClockDivide;
    colon = true;
  } else if (_currentState == ModuleState::Duty) {
    displayState = DisplayState::Duty;
  } else if (_currentState == ModuleState::ModStyle) {
    displayState = DisplayState::ModStyle;
  } else if (_currentState == ModuleState::ClockStop) {
    displayState = DisplayState::ClockStop;
  } else if (_currentState == ModuleState::Default) {
    if (_temporaryDisplayState == TemporaryDisplayState::None) {
      displayState = DisplayState::Default;
      colon = true;
    } else if (_temporaryDisplayState == TemporaryDisplayState::BeatsEqualDivs) {
      displayState = DisplayState::BeatsEqualDivs;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Calibrate) {
      displayState = DisplayState::Calibration;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Countdown && outs.modulationPending) {
      displayState = DisplayState::Countdown;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Done) {
      displayState = DisplayState::Done;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Tempo) {
      displayState = DisplayState::Tempo;
    }
  }

  int outValue[4];
  int outDecimal[4] = { 0, 0, 0, 0 };

  switch (displayState) {
    case DisplayState::Tempo:
      outValue[0] = (displayableTempo) / 1000;
      outValue[1] = ((displayableTempo) / 100) % 10;
      outValue[2] = ((displayableTempo) / 10) % 10;
      outDecimal[2] = tempoDecimal;
      outValue[3] = displayableTempo % 10;
      break;
    case DisplayState::Default:
      outValue[0] = activeDiv / 10;
      outValue[1] = activeDiv % 10;
      // outDecimal[1] = this->outs.subdivision;
      outDecimal[1] = 0;
      outValue[2] = activeBeats / 10;
      outValue[3] = activeBeats % 10;
      // outDecimal[3] = this->outs.beat;
      outDecimal[3] = 0;
      break;
    case DisplayState::Pop:
      outValue[0] = (int)SpecialDigits::Dash;
      outValue[1] = (int)SpecialDigits::Dash;
      outValue[2] = (int)SpecialDigits::Dash;
      outValue[3] = (int)SpecialDigits::Dash;
      break;
    case DisplayState::InputClockDivide:
      outValue[0] = (int)SpecialDigits::Nothing;
      outValue[1] = (int)1;
      outValue[2] = activeState.inputClockDivider / 10;
      if (outValue[2] == 0)
        outValue[2] = (int)SpecialDigits::Nothing;
      outValue[3] = activeState.inputClockDivider % 10;
      break;
    case DisplayState::BeatMode:
      if (activeState.beatInputResetMode) {
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = (int) SpecialDigits::R;
        outValue[2] = (int) 5;
        outValue[3] = (int) SpecialDigits::T;
      } else {
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = (int) SpecialDigits::D;
        outValue[2] = (int) SpecialDigits::E;
        outValue[3] = (int) SpecialDigits::F;
      }
      break;
    case DisplayState::BeatMult:
      outValue[0] = (int) activeState.beatMult;
      outValue[1] = (int) SpecialDigits::P;
      outValue[2] = (int) SpecialDigits::P;
      outValue[3] = (int) SpecialDigits::N;
      break;
    case DisplayState::DivMult:
      outValue[0] = (int) activeState.divMult;
      outValue[1] = (int) SpecialDigits::P;
      outValue[2] = (int) SpecialDigits::P;
      outValue[3] = (int) SpecialDigits::N;
      break;
    case DisplayState::Duty:
      if (activeState.fixedDutyCycle) {
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = 0;
        outDecimal[1] = 1;
        outValue[2] = 0;
        outValue[3] = 1;
      } else {
        colon = true;
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = 1;
        outValue[2] = 2;
        outValue[3] = (int) SpecialDigits::Nothing;
      }
      break;
    case DisplayState::ModStyle:
      outValue[0] = modStyleNames[(int) activeState.modulationStyle][0];
      outValue[1] = modStyleNames[(int) activeState.modulationStyle][1];
      outValue[2] = modStyleNames[(int) activeState.modulationStyle][2];
      outValue[3] = modStyleNames[(int) activeState.modulationStyle][3];
      break;
    case DisplayState::ClockStop:
      if (activeState.clockStop == 0) {
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = 0;
        outValue[2] = (int) SpecialDigits::F;
        outValue[3] = (int) SpecialDigits::F;
      } else if (activeState.clockStop == 1) {
        outValue[0] = (int) SpecialDigits::Nothing;
        outValue[1] = (int) SpecialDigits::Nothing;
        outValue[2] = 0;
        outValue[3] = (int) SpecialDigits::N;
      }
      break;
    case DisplayState::Countdown:
      outValue[0] = countdownSampleAndHold / 1000;
      if (outValue[0] == 0)
        outValue[0] = (int)SpecialDigits::Nothing;
      outValue[1] = countdownSampleAndHold / 100;
      if (outValue[1] == 0)
        outValue[1] = (int)SpecialDigits::Nothing;
      outValue[2] = countdownSampleAndHold / 10;
      if (outValue[2] == 0)
        outValue[2] = (int)SpecialDigits::Nothing;
      outValue[3] = countdownSampleAndHold % 10;
      break;
    case DisplayState::BeatsEqualDivs:
      outValue[0] = (int) SpecialDigits::Nothing;
      outValue[1] = (int) SpecialDigits::B;
      outValue[2] = (int) SpecialDigits::Equals;
      outValue[3] = (int) SpecialDigits::D;
      break;
    case DisplayState::Preset:
      outValue[0] = (int) SpecialDigits::P;
      outValue[1] = (int) targetPresetIndex + 1;
      outValue[2] = (int) SpecialDigits::Nothing;
      outValue[3] = presetAction == PresetAction::None ?
        (int)SpecialDigits::Nothing :
        presetAction == PresetAction::Recall ?
        (int)SpecialDigits::R :
        5;
      break;
    case DisplayState::Calibration:
      outValue[0] = (int) SpecialDigits::C;
      outValue[1] = (int) SpecialDigits::A;
      outValue[2] = (int) SpecialDigits::L;
      outValue[3] = (int) 1;
      break;
    case DisplayState::Done:
      outValue[0] = (int) SpecialDigits::D;
      outValue[1] = (int) 0;
      outValue[2] = (int) SpecialDigits::N;
      outValue[3] = (int) SpecialDigits::E;
      break;
    case DisplayState::ParamMenu:
      outValue[0] = paramMenuNames[_menuIndex][0];
      outValue[1] = paramMenuNames[_menuIndex][1];
      outValue[2] = paramMenuNames[_menuIndex][2];
      outValue[3] = paramMenuNames[_menuIndex][3];
      break;
  }

  value = outValue[digitCounter];
  decimal = outDecimal[digitCounter];
  hardware.sevenSegmentDisplay.process(digitCounter, value, decimal, colon);
}

void Module::_displayTemporaryWithTimer(Module::TemporaryDisplayState display, Timer *timer, int maxTime)
{
  if (_temporaryDisplayTimer != nullptr) {
    _temporaryDisplayTimer->clear();
  }

  _temporaryDisplayState = display;
  if (timer != nullptr) {
    timer->start(maxTime);
    _temporaryDisplayTimer = timer;
  }
}

void Module::_displayLatchLEDs()
{
  if (activeState.beat_latch) {
    if (activeState.beats != messd.beatsPerMeasure) {
      if (!_beatLatchFlashTimer.active()) {
        _beatLatchFlashTimer.start(LATCH_PULSE_TIME, -1);
      }
    } else {
      _beatLatchFlashTimer.clear();
    }
  } else {
    _beatLatchFlashState = false;
  }

  if (activeState.div_latch) {
    if (activeState.subdivisions != messd.subdivisionsPerMeasure) {
      if (!_divLatchFlashTimer.active()) {
        _divLatchFlashTimer.start(LATCH_PULSE_TIME, -1);
      }
    } else {
      _divLatchFlashTimer.clear();
    }
  } else {
    _divLatchFlashState = false;
  }

  bool beatLatchDisplay = activeState.beat_latch;
  if (activeState.beats != messd.beatsPerMeasure) {
    beatLatchDisplay = _beatLatchFlashState;
  }
  bool divLatchDisplay = activeState.div_latch;
  if (activeState.subdivisions != messd.subdivisionsPerMeasure) {
    divLatchDisplay = _divLatchFlashState;
  }

  leds_sr_val[(uint8_t)LEDNames::BeatLatchLED] = beatLatchDisplay ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::DivLatchLED] = divLatchDisplay ? LOW : HIGH;
}

void Module::HardwareRead(messd_ins_t *ins, messd_outs_t *outs){};
void Module::HardwareWrite(messd_ins_t *ins, messd_outs_t *outs){};

void Module::initHardware() {
  Serial.begin(9600);

  thresh_init(&beatSwitchThreshold, BEAT_INPUT_THRESH, BEAT_INPUT_HIST);

// Enable interrupts for the clock input pin
#if USING_MBED_RPI_PICO
  attachInterrupt(digitalPinToInterrupt(clockIn), &clockPinInterrupt, CHANGE);
#else
  cli();
  PCICR |= 0b00000011;  // Enables Port B Pin Change Interrupts
  PCMSK0 |= 0b00010000; // PB4
  sei();
#endif

  // Analog read pin for the digital mux
  /* hardware.DIGITAL_MUX_IN.PinInit(); */

  // Digital read for the dedicated mod switch pin
  /* hardware.MODSWITCH.PinInit(); */
  pinMode(hardware.MODSWITCH.address, hardware.MODSWITCH.type);

  // Clock pins
  // TODO - make part of Hardware class (Does it really have to be global?)
  pinMode(clockIn, INPUT);
  pinMode(clockJackSwitch, INPUT);

  for (int i = 0; i < 8; i++) {
    output_sr_val[i] = 0;
    leds_sr_val[i] = 0;
  }

  // Start the timer for the accurate clock
  float timerFreqHz = tapTempoOut * 100.0f /
                      30.0f; // double speed because it alternates at 2z Hz

#if USING_MBED_RPI_PICO
  ITimer0.attachInterrupt(timerFreqHz, TimerHandler);

  _display();

#else
  ITimer1.init();
  ITimer2.init();
  ITimer1.setFrequency(timerFreqHz, TimerHandler);
#endif

}

Module::Module()
: _nonVolatileStorage()
, _divCVQuantizer(DIV_INPUT_MIN, DIV_INPUT_MID, DIV_INPUT_MAX, -(beatsDivMax - beatsDivMin / 2), (beatsDivMax - beatsDivMin / 2), 0.1f)
, _beatCVQuantizer(BEAT_INPUT_MIN, BEAT_INPUT_MID, BEAT_INPUT_MAX, -(beatsDivMax - beatsDivMin / 2), (beatsDivMax - beatsDivMin / 2), 0.1f)
, _beatsEqualsDivTimer(std::bind(&Module::_clearTemporaryDisplayCallback, this, _1))
, _beatLatchFlashTimer((std::bind(&Module::_beatLatchTimerCallback, this, _1)))
, _divLatchFlashTimer((std::bind(&Module::_divLatchTimerCallback, this, _1)))
, _presetDisplayTimer((std::bind(&Module::_presetTimerCallback, this, _1)))
, _tempoDisplayTimer((std::bind(&Module::_clearTemporaryDisplayCallback, this, _1)))
, _calibrateDisplayTimer(std::bind(&Module::_clearTemporaryDisplayCallback, this, _1))
, _countdownDisplayTimer(std::bind(&Module::_clearTemporaryDisplayCallback, this, _1))
, _doneDisplayTimer(std::bind(&Module::_clearTemporaryDisplayCallback, this, _1))
, _beatButtonHoldTimer(std::bind(&Module::_beatButtonTimerCallback, this, _1))
, _divButtonHoldTimer(std::bind(&Module::_divButtonTimerCallback, this, _1))
, _paramMenuDisplayTimer(std::bind(&Module::_paramMenuTimerCallback, this, _1))
{
  MS_init(&this->messd);
};

void Module::process(float microsDelta) {

  if (!_nonVolatileStorageInitialized) {
    _initializeFromSavedData();
  }

  isClockInternal = !digitalRead(clockJackSwitch);
  uint8_t clockInput = isClockInternal ? lastInternalClockState : lastExternalClock;
  float ratio = (float) this->messd.tempoDivide / (float) this->messd.tempoMultiply;

  hardware.analogMux.process();
  isRoundTripMode = hardware.analogMux.getOutput(AnalogMux.ROUND_SWITCH) < (MAX_VOLTAGE >> 1);
  _processCalibration(microsDelta);

  hardware.digitalMux.process();
  _processTapTempo(microsDelta);
  _processEncoders(ratio);

  _processModSwitch(microsDelta);
  _processBeatDivSwitches(microsDelta);

  uint16_t rawBeatInput = hardware.analogMux.getOutput(AnalogMux.BEAT_INPUT);
  uint16_t invRawBeatInput = min(BEAT_INPUT_MAX, max(BEAT_INPUT_MIN, rawBeatInput));
  invRawBeatInput = BEAT_INPUT_MAX - invRawBeatInput + BEAT_INPUT_MIN; // invert
  char beatThreshOutput = 0;
  thresh_process(&beatSwitchThreshold, &invRawBeatInput, &beatThreshOutput);
  uint8_t didReset = 0;
  this->ins.resetBeatCount = 0;
  if (activeState.beatInputResetMode && (beatThreshOutput && !this->lastBeatInputValue)) {
    this->ins.resetBeatCount = 1;
    didReset = 1;
    Serial.println("reset");
  }
  this->lastBeatInputValue = beatThreshOutput;

  // Compute the final value for subdivisions and beats based on modulation
  // inputs and attenuverters
  // Handle the slight asymmetry in the div input
  int divInput = hardware.analogMux.getOutput(AnalogMux.DIVIDE_INPUT);
  divInput = max(DIV_INPUT_MIN, min(DIV_INPUT_MAX, divInput));
  divInput = DIV_INPUT_MAX - divInput + DIV_INPUT_MIN; // invert
  int32_t qdivInput = _divCVQuantizer.process(divInput);

  static float divAttenuvertRange = (DIV_ATV_MAX - DIV_ATV_MIN);
  int divAttenuvertInput = hardware.analogMux.getOutput(AnalogMux.DIVIDE_ATV);
  divAttenuvertInput = max(DIV_ATV_MIN, min(DIV_ATV_MAX, divAttenuvertInput));
  float divAttenuvert =
      (float)(divAttenuvertInput - DIV_ATV_MIN) / divAttenuvertRange;
  divAttenuvert = 2.0f * (divAttenuvert - 0.5);
  divAttenuvert *= 1.5; // a little boost
  int divOffset = qdivInput * divAttenuvert;

  float divBase = activeState.subdivisions;
  activeDiv = min(beatsDivMax, max(beatsDivMin, divBase + divOffset));

  int beatsOffset = 0;
  int beatsBase = activeState.beats;
  if (!activeState.beatInputResetMode) {
    int beatSigInput = max(BEAT_INPUT_MIN, min(BEAT_INPUT_MAX, invRawBeatInput));
    beatsOffset = _beatCVQuantizer.process(beatSigInput);
  }
  activeBeats = min(beatsDivMax, max(beatsDivMin, beatsBase + beatsOffset));

  this->ins.resyncToExternal = !isClockInternal;
  this->ins.tempo = tapTempoOut;
  this->ins.beatsPerMeasure = activeBeats;
  this->ins.subdivisionsPerMeasure = activeDiv;
  this->ins.phase = 0; // unused
  this->ins.ext_clock = clockInput == HIGH;
  this->ins.useTenMillisecondWidth = activeState.fixedDutyCycle;
  this->ins.modulationStyle = activeState.modulationStyle;
  this->ins.beatOutputMultiplier = activeState.beatMult;
  this->ins.divOutputMultiplier = activeState.divMult;

  this->ins.modulationSignal =
      hardware.analogMux.getOutput(AnalogMux.MOD_INPUT) < (calibratedState.modInputMid + MOD_INPUT_THRESH);
  this->ins.modulationSwitch = this->modSwitch == LOW; // active low
  this->ins.latchBeatChangesToDownbeat = activeState.beat_latch;
  this->ins.latchDivChangesToDownbeat = activeState.div_latch;

  this->ins.latchModulationToDownbeat =
      hardware.analogMux.getOutput(AnalogMux.LATCH_SWITCH) > (MAX_VOLTAGE >> 1);
  this->ins.invert = 0; // unused
  this->ins.isRoundTrip = isRoundTripMode;

  if (this->ins.reset || (this->modHoldTime > MOD_BUTTON_RESET_TIME_MICROS && this->canTriggerReset)) {
    this->ins.reset = true;
    this->canTriggerReset = false;
  } else {
    this->ins.reset = false;
  }

  // compute wrap
  static int truncAttenuverterRange = (TRUNC_ATV_MAX - TRUNC_ATV_MIN);
  static int truncInputRange = (TRUNC_INPUT_MAX - TRUNC_INPUT_MIN);
  int truncAttenuverterInput = hardware.analogMux.getOutput(AnalogMux.TRUNCATE_ATV);
  truncAttenuverterInput = min(TRUNC_ATV_MAX, max(TRUNC_ATV_MIN, truncAttenuverterInput));
  float baseTruncation =
      (float)(truncAttenuverterInput - TRUNC_ATV_MIN) /
      (float)truncAttenuverterRange;
  int truncationInput = hardware.analogMux.getOutput(AnalogMux.TRUNCATE_INPUT);
  truncationInput = min(TRUNC_INPUT_MAX, max(TRUNC_INPUT_MIN, truncationInput));
  truncationInput = TRUNC_INPUT_MAX - truncationInput  + TRUNC_INPUT_MIN; // invert
  float truncationOffset = 0.0f;
  if (truncationInput > calibratedState.truncInputMid) {
    truncationOffset = (float)(truncationInput - calibratedState.truncInputMid) / ((float) (TRUNC_INPUT_MAX - calibratedState.truncInputMid));
  } else {
    truncationOffset = (float)(truncationInput - calibratedState.truncInputMid) / ((float) (calibratedState.truncInputMid - TRUNC_INPUT_MIN));
  }
  truncationOffset *= 0.8; // right?

  baseTruncation = fmax(0.0, fmin(1.0, baseTruncation + truncationOffset));
  this->ins.truncation = baseTruncation;

  this->ins.pulseWidth = 0.5;

  // Cheat and pass in the measured period time directly
  if (this->lastRecordedHighClockTime != lastHighClockTime) {

    if (this->lastRecordedHighClockTime > 0 && lastHighClockTime > 0)
      hasProcessedHighClock = true;

    if (hasProcessedHighClock) {
      if (lastHighClockTime > this->lastRecordedHighClockTime) {
        measuredPeriod = lastHighClockTime - this->lastRecordedHighClockTime;
      } else {
        measuredPeriod = (4294967295 - this->lastRecordedHighClockTime) +
                         lastHighClockTime; // wraparound
      }
    }
    this->lastRecordedHighClockTime = lastHighClockTime;
  }

  if (isClockInternal) {
    this->ins.cheatedMeasuredPeriod = 60000000.0 / (double) tapTempoOut;
  } else {
    this->ins.cheatedMeasuredPeriod = measuredPeriod;
  }

  // Serial.println(this->ins.cheatedMeasuredPeriod);
  // Serial.println(measuredPeriod);

  // Calculate offset since last process time
  unsigned long now = micros();
  unsigned long offset;
  if (now > this->lastProcessTime) {
    offset = now - this->lastProcessTime;
  } else {
    offset = (4294967295 - this->lastProcessTime) + now; // wraparound
  }
  this->lastProcessTime = now;
  this->ins.delta = ((float)offset) / 1000.0;

  hardware.digitalMux.process();
  _processTapTempo(microsDelta);
  _processEncoders(ratio);

  MS_process(&this->messd, &this->ins, &this->outs);

  float beatPeriod = 30000000.0f * messd.tempoDivide / (messd.measuredTempo * messd.tempoMultiply);
  float dividePeriod = beatPeriod * messd.beatsPerMeasure / messd.subdivisionsPerMeasure;
  float beatLEDBuffer = fminf(LED_BUFFER_MICROS, beatPeriod);
  float divLEDBuffer = fminf(LED_BUFFER_MICROS, dividePeriod);

  if (!this->lastDownbeat && this->outs.downbeat) {
    if (this->messd.modulationPending && this->messd.inRoundTripModulation) {
      _displayTemporaryWithTimer(
        TemporaryDisplayState::Countdown,
        &_countdownDisplayTimer,
        COUNTDOWN_DISPLAY_TIME
      );
      this->countdownSampleAndHold = this->outs.countdown;
    }
  }
  this->lastDownbeat = this->outs.downbeat;

  if (this->outs.eom) {
    _ledBuffers[(int) LEDOutputs::EoM] = 0;
    this->animateModulateButtonTime = 0.0f;
    this->modButtonFlashTimer = 0.0;
    this->modButtonFlashCount = 0;

    this->activeState.subdivisions = this->ins.subdivisionsPerMeasure;
    this->activeState.beats = this->ins.beatsPerMeasure;
  }

  if (outs.downbeat) {
    _ledBuffers[(int) LEDOutputs::Down] = 0;
  }
  if (outs.truncate) {
    _ledBuffers[(int) LEDOutputs::Truncate] = 0;
  }
  if (outs.subdivision) {
    _ledBuffers[(int) LEDOutputs::Divide] = 0;
  }
  if (outs.beat) {
    _ledBuffers[(int) LEDOutputs::Beat] = 0;
  }

  if (this->outs.modulationRequestSkipped) {
    this->modButtonFlashTimer = 0.0;
    this->modButtonFlashCount = 0.0;
    this->modulationButtonIgnored = true;
    _displayTemporaryWithTimer(
      TemporaryDisplayState::BeatsEqualDivs,
      &_beatsEqualsDivTimer,
      OTHER_DISPLAY_TIME
    );
  }
  if (this->modSwitch == HIGH) {
    this->modulationButtonIgnored = false;
  }
  // Animate the modulation button
  bool modButtonOn = false;
  if (this->modSwitch == LOW) {
    if (this->modulationButtonIgnored) {
      if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
        modButtonOn = this->modButtonFlashCount % 2 > 0;
      }
    } else {
      modButtonOn = true;
    }
  } else if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
    // Serial.println(modButtonFlashCount);
    modButtonOn = this->modButtonFlashCount % 2 > 0;
  } else if (this->outs.modulationPending) {
    modButtonOn = this->animateModulateButtonTime < MOD_BUTTON_STROBE_SLOW;
    this->animateModulateButtonTime += microsDelta;
    if (this->animateModulateButtonTime > (2.0f * MOD_BUTTON_STROBE_SLOW)) {
      this->animateModulateButtonTime = fmodf(this->animateModulateButtonTime,
                                              (2.0f * MOD_BUTTON_STROBE_SLOW));
    }
  } else if (this->outs.inRoundTripModulation) {
    modButtonOn = true;
  }

  hardware.digitalMux.process();
  _processTapTempo(microsDelta);
  _processEncoders(ratio);

  // Configure outputs
  output_sr_val[(uint8_t)OutputNames::Nothing] = HIGH;
  output_sr_val[(uint8_t)OutputNames::TruncateLED] =
    (activeState.fixedDutyCycle ? (_ledBuffers[(int) LEDOutputs::Truncate] < divLEDBuffer) : outs.truncate) ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DivLED] =
    (activeState.fixedDutyCycle ? (_ledBuffers[(int) LEDOutputs::Divide] < divLEDBuffer) : outs.subdivision) ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::EoMOutput] =
      _ledBuffers[(int) LEDOutputs::EoM] < EOM_BUFFER_MICROS ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::TruncateOutput] =
      this->outs.truncate ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DivOutput] =
      this->outs.div_ppqn ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DownbeatOutput] =
      this->outs.downbeat ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::BeatOutput] =
      this->outs.beat_ppqn ? LOW : HIGH;

  hardware.moduleOuts.process(this->output_sr_val, 8, true);

  // Configure LEDs
  leds_sr_val[(uint8_t)LEDNames::Nothing] = HIGH;
  leds_sr_val[(uint8_t)LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::EoMLED] =
      _ledBuffers[(int) LEDOutputs::EoM] < LED_BUFFER_MICROS ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::ClockLEDButton] =
      clockInput == HIGH ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::DownbeatLED] =
    (activeState.fixedDutyCycle ? (_ledBuffers[(int) LEDOutputs::Down] < beatLEDBuffer) : outs.downbeat) ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::BeatLED] =
    (activeState.fixedDutyCycle ? (_ledBuffers[(int) LEDOutputs::Beat] < beatLEDBuffer) : outs.beat) ? LOW : HIGH;
  _displayLatchLEDs();

  hardware.moduleLEDs.process(this->leds_sr_val, 8, true);

  this->scaledTempo = this->outs.scaledTempo;

  // about 400 msec
  _display();

  hardware.digitalMux.process();
  _processTapTempo(microsDelta);
  _processEncoders(ratio);
  this->ins.reset = false;

  for (int i = 0; i < (unsigned int) LEDOutputs::LEDOutputs_LENGTH; i++) {
    _ledBuffers[i] += microsDelta;
    if (_ledBuffers[i] > (2 * LED_BUFFER_MICROS)) {
      _ledBuffers[i] = 2 * LED_BUFFER_MICROS;
    }
  }

  if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
    this->modButtonFlashTimer += microsDelta;
    if (this->modButtonFlashTimer > MOD_BUTTON_FLASH_TIME) {
      this->modButtonFlashCount++;
      this->modButtonFlashTimer =
          (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT)
              ? 0.0
              : MOD_BUTTON_FLASH_TIME;
    }
  }

  for (int i = 0; i < NUM_TIMERS; i++) {
    auto timer = _timers[i];
    if (timer->active()) {
      timer->tick(microsDelta);
    }
  }
};
