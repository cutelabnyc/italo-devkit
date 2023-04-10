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

void Module::_tempoDisplayTimerCallback(float progress)
{
  if (progress >= 1.0f) {
    if (_currentState == ModuleState::Tempo) {
      _currentState = ModuleState::Default;
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
      // TODO
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
      // TODO: Handle the param menu stuff here

      activeState.inputClockDivider += inc;
      this->inputClockDivDisplayTime = 0.0f;
      if (activeState.inputClockDivider < Module::inputClockDivideMin) {
        activeState.inputClockDivider = Module::inputClockDivideMin;
      } else if (activeState.inputClockDivider > Module::inputClockDivideMax) {
        activeState.inputClockDivider = Module::inputClockDivideMax;
      }

      atomicInputClockDivider = activeState.inputClockDivider;
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
      if (delta > 2000000) {
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

void Module::_processBeatDivSwitches(float microsDelta) {
  // div switch
  if (hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH) == LOW) {
    if (_currentState == ModuleState::Default) {
      if (div_switch_state_prev == HIGH) {
        initial_div_latch = activeState.div_latch;
        activeState.div_latch = !activeState.div_latch;
      } else {
        divHoldTime += microsDelta;
        if (divHoldTime >= DIV_BUTTON_HOLD_TIME) {
          activeState.div_latch = initial_div_latch;

          if (hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH) == LOW) {
            activeState.beat_latch = initial_beat_latch;
            if (calibrationPossible) {
              doCalibrate = true;
              _displayTemporaryWithTimer(TemporaryDisplayState::Calibrate, &_calibrateDisplayTimer, OTHER_DISPLAY_TIME);
            } else {
              _currentState = ModuleState::Preset;
              _presetDisplayTimer.start(PRESET_DISPLAY_TIME);
            }
          } else {
            inputClockDivDisplayTime = 0;
          }

          divHoldTime = 0;
        }
      }
    } else if (_currentState == ModuleState::Preset) {
      if (div_switch_state_prev == HIGH) {
        presetAction = PresetAction::None;
        _currentState = ModuleState::Default;
        _presetDisplayTimer.clear();
      }
    }
  } else {
    inputClockDivDisplayTime += microsDelta;
    if (inputClockDivDisplayTime > OTHER_DISPLAY_TIME + 1)
      inputClockDivDisplayTime = OTHER_DISPLAY_TIME + 1;
    divHoldTime = 0.0f;
    calibrationPossible = false;
  }
  div_switch_state_prev = hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH);

  // beat switch
  if (hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH) == LOW) {
    if (_currentState == ModuleState::Default) {
      if (beat_switch_state_prev == HIGH) {
        initial_beat_latch = activeState.beat_latch;
        activeState.beat_latch = !activeState.beat_latch;
      } else {
        beatHoldTime += microsDelta;
        if (beatHoldTime > BEAT_BUTTON_HOLD_TIME) {
          if (hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH) == LOW) {
            activeState.beat_latch = initial_beat_latch;
            activeState.div_latch = initial_div_latch;
            if (calibrationPossible) {
              doCalibrate = true;
              _displayTemporaryWithTimer(TemporaryDisplayState::Calibrate, &_calibrateDisplayTimer, OTHER_DISPLAY_TIME);
            } else {
              _currentState = ModuleState::Preset;
              _presetDisplayTimer.start(PRESET_DISPLAY_TIME);
            }

          } else if (canSwitchBeatInputModes) {
            activeState.beat_latch = initial_beat_latch;
            activeState.beatInputResetMode = !activeState.beatInputResetMode;
            canSwitchBeatInputModes = false;
            beatModeDisplayTime = 0.0f;
          }

          beatHoldTime = 0;
        }
      }
    } else if (_currentState == ModuleState::Preset) {
      if (beat_switch_state_prev == HIGH) {

        if (presetAction == PresetAction::Recall) {
          _nonVolatileStorage.readPreset(targetPresetIndex, &activeState);
          doneDisplayTimer = 0;
        } else if (presetAction == PresetAction::Store) {
          _nonVolatileStorage.storePreset(targetPresetIndex, &activeState);
          doneDisplayTimer = 0;
        }

        presetAction = PresetAction::None;
        _currentState = ModuleState::Default;
        _presetDisplayTimer.clear();
      }
    }
  } else {
    beatModeDisplayTime += microsDelta;
    if (beatModeDisplayTime > OTHER_DISPLAY_TIME + 1)
      beatModeDisplayTime = OTHER_DISPLAY_TIME + 1;
    beatHoldTime = 0.0f;
    canSwitchBeatInputModes = true;
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
  BeatMode,
  Countdown,
  BeatsEqualDivs,
  Preset,
  Calibration,
  Done
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
    // TODO
  } else if (_currentState == ModuleState::Default) {
    if (_temporaryDisplayState == TemporaryDisplayState::None) {
      displayState = DisplayState::Default;
      colon = true;
    } else if (_temporaryDisplayState == TemporaryDisplayState::BeatsEqualDivs) {
      displayState = DisplayState::BeatsEqualDivs;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Calibrate) {
      displayState = DisplayState::Calibration;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Countdown) {
      displayState = DisplayState::Countdown;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Done) {
      displayState = DisplayState::Done;
    } else if (_temporaryDisplayState == TemporaryDisplayState::Tempo) {
      displayState = DisplayState::Tempo;
    }
  }

  switch (digitCounter) {
  case 0:
    if (displayState == DisplayState::Tempo) {
      value = (displayableTempo) / 1000;
    } else if (displayState == DisplayState::Default) {
      value = activeDiv / 10;
    } else if (displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (displayState == DisplayState::InputClockDivide) {
      value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::BeatMode) {
      value =
          (int)(activeState.beatInputResetMode ? SpecialDigits::Nothing : SpecialDigits::B);
    } else if (displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 1000;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::Preset) {
      value = (int)SpecialDigits::P;
    } else if (displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::C;
    } else if (displayState == DisplayState::Done) {
      value = (int)SpecialDigits::D;
    }
    break;
  case 1:
    if (displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 100) % 10;
    } else if (displayState == DisplayState::Default) {
      value = activeDiv % 10;
      decimal = this->outs.subdivision;
    } else if (displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (displayState == DisplayState::InputClockDivide) {
      value = (int)1;
    } else if (displayState == DisplayState::BeatMode) {
      value = (int)(activeState.beatInputResetMode ? SpecialDigits::R : SpecialDigits::E);
    } else if (displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 100;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::D;
    } else if (displayState == DisplayState::Preset) {
      value = targetPresetIndex + 1;
    } else if (displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::A;
    } else if (displayState == DisplayState::Done) {
      value = 0;
    }
    break;
  case 2:
    if (displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 10) % 10;
      decimal = tempoDecimal;
    } else if (displayState == DisplayState::Default) {
      value = activeBeats / 10;
    } else if (displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (displayState == DisplayState::InputClockDivide) {
      value = activeState.inputClockDivider / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::BeatMode) {
      value = (activeState.beatInputResetMode ? 5 : (int)SpecialDigits::A);
    } else if (displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Equals;
    } else if (displayState == DisplayState::Preset) {
      value = (int)SpecialDigits::Nothing;
    } else if (displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::L;
    } else if (displayState == DisplayState::Done) {
      value = (int)SpecialDigits::N;
    }
    break;
  case 3:
    if (displayState == DisplayState::Tempo) {
      value = displayableTempo % 10;
    } else if (displayState == DisplayState::Default) {
      value = activeBeats % 10;
      decimal = this->outs.beat;
    } else if (displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (displayState == DisplayState::InputClockDivide) {
      value = activeState.inputClockDivider % 10;
    } else if (displayState == DisplayState::BeatMode) {
      value = (int)SpecialDigits::T;
    } else if (displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold % 10;
    } else if (displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::B;
    } else if (displayState == DisplayState::Preset) {
      value = presetAction == PresetAction::None ?
        (int)SpecialDigits::Nothing :
        presetAction == PresetAction::Recall ?
        (int)SpecialDigits::R :
        5;
    } else if (displayState == DisplayState::Calibration) {
      value = 1;
    } else if (displayState == DisplayState::Done) {
      value = (int)SpecialDigits::E;
    }
    break;
  }

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
  }

  if (activeState.div_latch) {
    if (activeState.subdivisions != messd.subdivisionsPerMeasure) {
      if (!_divLatchFlashTimer.active()) {
        _divLatchFlashTimer.start(LATCH_PULSE_TIME, -1);
      }
    } else {
      _divLatchFlashTimer.clear();
    }
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

  this->eomBuffer = EOM_BUFFER_MICROS;

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
, _tempoDisplayTimer((std::bind(&Module::_tempoDisplayTimerCallback, this, _1)))
, _calibrateDisplayTimer(std::bind(&Module::_clearTemporaryDisplayCallback, this, _1))
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
    // Serial.print("b: ");
    // Serial.print(rawBeatInput);
    // Serial.print(", ");
    int beatSigInput = max(BEAT_INPUT_MIN, min(BEAT_INPUT_MAX, rawBeatInput));
    beatsOffset = _beatCVQuantizer.process(beatSigInput);
    // Serial.println(beatsOffset);
  }
  activeBeats = min(beatsDivMax, max(beatsDivMin, beatsBase + beatsOffset));

  this->ins.resyncToExternal = !isClockInternal;
  this->ins.tempo = tapTempoOut;
  this->ins.beatsPerMeasure = activeBeats;
  this->ins.subdivisionsPerMeasure = activeDiv;
  this->ins.phase = 0; // unused
  this->ins.ext_clock = clockInput == HIGH;

  this->ins.modulationSignal =
      hardware.analogMux.getOutput(AnalogMux.MOD_INPUT) < (calibratedState.modInputMid + MOD_INPUT_THRESH);
  this->ins.modulationSwitch = this->modSwitch == LOW; // active low
  this->ins.latchBeatChangesToDownbeat = activeState.beat_latch;
  this->ins.latchDivChangesToDownbeat = activeState.div_latch;

  this->ins.latchModulationToDownbeat =
      hardware.analogMux.getOutput(AnalogMux.LATCH_SWITCH) > (MAX_VOLTAGE >> 1);
  this->ins.invert = 0; // unused
  this->ins.isRoundTrip = isRoundTripMode;

  if (this->modHoldTime > MOD_BUTTON_RESET_TIME_MICROS && this->canTriggerReset) {
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
  truncationInput = min(TRUNC_INPUT_MAX, max(TRUNC_ATV_MIN, truncationInput));
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

  // static int lastModulationPending = false;
  // if (!lastModulationPending && this->messd.modulationPending) {

  //     uint8_t originalBeatsPerMeasure = messd.originalBeatsPerMeasure == 0 ?
  //     messd.beatsPerMeasure : messd.originalBeatsPerMeasure; float
  //     originalMeasurePhase = (messd.originalBeatCounter +
  //     messd.rootClockPhase) / originalBeatsPerMeasure; float
  //     scaledMeasurePhase = (messd.scaledBeatCounter + messd.scaledClockPhase)
  //     / messd.beatsPerMeasure; float scaleFactor = (1.0f -
  //     scaledMeasurePhase) / (1.0f - originalMeasurePhase); scaleFactor *=
  //     ((float) messd.tempoDivide) / ((float) messd.tempoMultiply);

  //     Serial.print(this->messd.originalBeatCounter);
  //     Serial.print("\t");
  //     Serial.print(originalBeatsPerMeasure);
  //     Serial.print("\t");
  //     Serial.print(this->messd.scaledBeatCounter);
  //     Serial.print("\t");
  //     Serial.print(this->messd.beatsPerMeasure);
  //     Serial.print("\t");
  //     Serial.print(originalMeasurePhase);
  //     Serial.print("\t");
  //     Serial.print(scaledMeasurePhase);
  //     Serial.print("\t");
  //     Serial.print(messd.tempoDivide);
  //     Serial.print("\t");
  //     Serial.print(messd.tempoMultiply);
  //     Serial.print("\t");
  //     Serial.println(scaleFactor);
  //     Serial.println("---");
  // }
  // lastModulationPending = this->messd.modulationPending;

  if (!this->lastDownbeat && this->outs.downbeat) {
    if (this->messd.modulationPending && this->messd.inRoundTripModulation) {
      this->countdownDisplayTime = 0;
      this->countdownSampleAndHold = this->outs.countdown;
    }
  }
  this->countdownDisplayTime += microsDelta;
  this->countdownDisplayTime =
      min(this->countdownDisplayTime, COUNTDOWN_DISPLAY_TIME);
  this->lastDownbeat = this->outs.downbeat;

  if (this->outs.eom) {
    this->eomBuffer = 0;
    this->animateModulateButtonTime = 0.0f;
    this->modButtonFlashTimer = 0.0;
    this->modButtonFlashCount = 0;

    this->activeState.subdivisions = this->ins.subdivisionsPerMeasure;
    this->activeState.beats = this->ins.beatsPerMeasure;

    // Serial.println(this->messd.rootClockPhase);
    // Serial.println(this->messd.scaledClockPhase);
    // Serial.println(this->messd.rootClockPhaseOffset);
    // Serial.println(this->messd.rootBeatCounter);
    // Serial.println(this->messd.scaledBeatCounter);
    // Serial.println(this->messd.tempoDivide);
    // Serial.println(this->messd.tempoMultiply);
    // float offsetRootMeasurePhase = (this->messd.rootClockPhase +
    // this->messd.rootBeatCounter + this->messd.rootClockPhaseOffset); if
    // (offsetRootMeasurePhase > this->messd.tempoDivide) offsetRootMeasurePhase
    // -= this->messd.tempoDivide;

    // float nextScaledClockPhase = (offsetRootMeasurePhase *
    // this->messd.tempoMultiply) / this->messd.tempoDivide;
    // nextScaledClockPhase = fmod(nextScaledClockPhase, 1.0f);
    // Serial.println(nextScaledClockPhase);
    // Serial.println("---");
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

  // Serial.print(this->outs.patternIndex);
  // Serial.print("\t");
  // Serial.print(this->outs.chunk);
  // Serial.print("\t");
  // Serial.print(this->outs.prescale);
  // Serial.print("\t");
  // Serial.print(this->outs.subchunk);
  // Serial.print("\t");
  // Serial.print(this->outs.scale);
  // Serial.print("\t");
  // Serial.println(this->outs.patternPhase);
  // Serial.println("---");

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
      this->outs.truncate ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DivLED] =
      this->outs.subdivision ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::EoMOutput] =
      this->eomBuffer < EOM_BUFFER_MICROS ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::TruncateOutput] =
      this->outs.truncate ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DivOutput] =
      this->outs.subdivision ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DownbeatOutput] =
      this->outs.downbeat ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::BeatOutput] =
      this->outs.beat ? LOW : HIGH;

  hardware.moduleOuts.process(this->output_sr_val, 8, true);

  // Configure LEDs
  leds_sr_val[(uint8_t)LEDNames::Nothing] = HIGH;
  leds_sr_val[(uint8_t)LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::EoMLED] =
      this->eomBuffer < EOM_LED_BUFFER_MICROS ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::ClockLEDButton] =
      clockInput == HIGH ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::DownbeatLED] =
      this->outs.downbeat ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
  _displayLatchLEDs();

  hardware.moduleLEDs.process(this->leds_sr_val, 8, true);

  this->scaledTempo = this->outs.scaledTempo;

  // about 400 msec
  _display();

  hardware.digitalMux.process();
  _processTapTempo(microsDelta);
  _processEncoders(ratio);

  this->eomBuffer += microsDelta;
  if (this->eomBuffer > 5000000)
    this->eomBuffer = 5000000;

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

  this->doneDisplayTimer += microsDelta;
    if (this->doneDisplayTimer > OTHER_DISPLAY_TIME * 2) {
    this->doneDisplayTimer = OTHER_DISPLAY_TIME;
  }

  for (int i = 0; i < NUM_TIMERS; i++) {
    auto timer = _timers[i];
    if (timer->active()) {
      timer->tick(microsDelta);
    }
  }
};
