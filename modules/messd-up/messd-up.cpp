#include "messd-up.hpp"
#include "timers.hpp"

#include <avr/interrupt.h>

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

// TODO: Figure out how to use this interrupt pin on rp2040
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

template <typename DataType>
NonVolatileStorage<DataType>::NonVolatileStorage() { }

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::loadLastPreset(DataType *data)
{
  Serial.println("Initializing nonvolatlie storage");

  if (!myFS.init()) {
    Serial.println("mount failed");
  }

  Serial.println("Initializing data");
  char buff[512];
  sprintf(buff, "%s", _indexFilename);
  int index;
  uint8_t status = read(buff, &index);

  if (!status) {
    Serial.println("Could not recover index of last saved preset");
    return status;
  }

  sprintf(buff, "/littlefs/preset%d.txt", index);
  status = read(buff, data);

  if (!status) {
    Serial.println("Could not load stored data, using defaults");
  }

  return status;
}

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::readPreset(int index, DataType *data)
{
  Serial.print("Reading preset at index ");
  Serial.println(index);

  char buff[128];
  sprintf(buff, "/littlefs/preset%d.txt", index);
  uint8_t status = read(buff, data);

  // Store the index of the last used file
  if (status) {
    sprintf(buff, "%s", _indexFilename);
    status = store(buff, &index);
  }

  return status;
}

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::storePreset(int index, DataType *data)
{
  Serial.print("Storing preset at index ");
  Serial.println(index);

  char buff[128];
  sprintf(buff, "/littlefs/preset%d.txt", index);
  uint8_t status = store(buff, data);

  // Store the index of the last used file
  if (status) {
    sprintf(buff, "%s", _indexFilename);
    status = store(buff, &index);
  }

  return status;
}

template <typename DataType>
template <typename T>
uint8_t NonVolatileStorage<DataType>::read(const char *path, T *outData)
{
  char buff[512];
  uint8_t status = 0;
  sprintf(buff, "Reading file at %s", path);
  Serial.println(buff);

  FILE *f = fopen(path, "r");

  if (!f) {
    Serial.println("Could not open file for reading");
  } else {
    size_t readBytes = fread(outData, 1, sizeof(T), f);
    if (readBytes >= sizeof(T)) {
      Serial.println("Read data successfully");
      status = 1;
    } else {
      Serial.println("Could not read data file");
    }

    fclose(f);
  }

  return status;
}

template <typename DataType>
template <typename T>
uint8_t NonVolatileStorage<DataType>::store(const char *path, T *data)
{
  char buff[512];
  uint8_t status = 0;
  sprintf(buff, "Storing file at %s", path);
  Serial.println(buff);

  FILE *f = fopen(path, "w");

  if (!f) {
    Serial.println("Could not open file for writing");
  } else {
    size_t writtenBytes = fwrite(data, 1, sizeof(T), f);
    if (writtenBytes >= sizeof(T)) {
      Serial.println("Wrote data successfully");
      status = 1;
    } else {
      Serial.println("Could not write data file");
    }

    fclose(f);
  }

  return status;
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

    // When you hold down the clock button, you set the tempo with the knobs
    if (this->clockSwitch == LOW && isClockInternal) {
      activeState.tapTempo += inc * incScale * 0.1;
      tapTempoOut += inc * incScale * 0.1;
      if (activeState.tapTempo < Module::tempoMin)
        activeState.tapTempo = tapTempoOut = Module::tempoMin;
      if (activeState.tapTempo > Module::tempoMax)
        activeState.tapTempo = tapTempoOut = Module::tempoMax;
    } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
      // No-op, the beat encoder doesn't do anything here
    } else if (presetDisplayTimer < PRESET_DISPLAY_TIME) {
      presetDisplayTimer = 0;
      if (presetAction == PresetAction::None || presetAction == PresetAction::Store) {
        presetAction = PresetAction::Recall;
      } else {
        presetAction = PresetAction::Store;
      }
    } else {
      this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
      this->beatModeDisplayTime = OTHER_DISPLAY_TIME;
      this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME;

      // You can't change beats when you're in a round trip modulation, in latch
      // mode
      if (!(messd.inRoundTripModulation && ins.latchModulationToDownbeat)) {
        activeState.beats += inc;
        this->latchPulseTimer = 0.0f;
        if (activeState.beats < beatsDivMin)
          activeState.beats = beatsDivMax;
        if (activeState.beats > beatsDivMax)
          activeState.beats = beatsDivMin;
      }
    }
  }

  inc = hardware.div.process(
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_A),
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_B));

  if (inc != 0) {
    // When you hold down the clock button, you set the tempo with the knobs
    if (this->clockSwitch == LOW && isClockInternal) {
      activeState.tapTempo += inc * 10 * incScale;
      tapTempoOut += inc * 10 * incScale;
      if (activeState.tapTempo < Module::tempoMin)
        activeState.tapTempo = tapTempoOut = Module::tempoMin;
      if (activeState.tapTempo > Module::tempoMax)
        activeState.tapTempo = tapTempoOut = Module::tempoMax;
    } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
      activeState.inputClockDivider += inc;
      this->inputClockDivDisplayTime = 0.0f;
      if (activeState.inputClockDivider < Module::inputClockDivideMin) {
        activeState.inputClockDivider = Module::inputClockDivideMin;
      } else if (activeState.inputClockDivider > Module::inputClockDivideMax) {
        activeState.inputClockDivider = Module::inputClockDivideMax;
      }

      atomicInputClockDivider = activeState.inputClockDivider;
    } else if (presetDisplayTimer < PRESET_DISPLAY_TIME) {
      presetDisplayTimer = 0;
      targetPresetIndex += inc;
      if (targetPresetIndex < 0) targetPresetIndex = 9 - ( abs(targetPresetIndex) % 9);
      if (targetPresetIndex >= 9) targetPresetIndex %= 9;
    } else {
      this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
      this->beatModeDisplayTime = OTHER_DISPLAY_TIME;
      this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME;

      activeState.subdivisions += inc;
      this->latchPulseTimer = 0.0f;
      if (activeState.subdivisions < beatsDivMin)
        activeState.subdivisions = beatsDivMax;
      if (activeState.subdivisions > beatsDivMax)
        activeState.subdivisions = beatsDivMin;
    }
  }
}

void Module::_processTapTempo(float microsDelta) {
  int nextClockSwitch = hardware.digitalMux.getOutput(DigitalMux.CLOCK_SWITCH);

  // for (int i = 0; i < 8; i++) {
  //   if (i != 0) Serial.print(",\t");
  //   Serial.print(hardware.digitalMux.getOutput(i));
  // }
  // Serial.println();

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

    if (nextClockSwitch != LOW) {
      this->tempoDisplayTime += microsDelta;
      if (this->tempoDisplayTime > TEMPO_DISPLAY_TIME * 2) {
        this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
      }
    } else {
      this->tempoDisplayTime = 0;
      this->inputClockDivDisplayTime = OTHER_DISPLAY_TIME;
    }
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
    if (presetDisplayTimer >= PRESET_DISPLAY_TIME) {
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
              calibrateDisplayTime = 0;
            } else {
              presetDisplayTimer = 0;
            }
          } else {
            inputClockDivDisplayTime = 0;
          }

          divHoldTime = 0;
        }
      }
    } else {
      if (div_switch_state_prev == HIGH) {
        presetDisplayTimer = PRESET_DISPLAY_TIME;
        presetAction = PresetAction::None;
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
    if (presetDisplayTimer >= PRESET_DISPLAY_TIME) {
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
              calibrateDisplayTime = 0;
            } else {
              presetDisplayTimer = 0;
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
    } else {
      if (beat_switch_state_prev == HIGH) {
        presetDisplayTimer = PRESET_DISPLAY_TIME;

        if (presetAction == PresetAction::Recall) {
          _nonVolatileStorage.readPreset(targetPresetIndex, &activeState);
          doneDisplayTimer = 0;
        } else if (presetAction == PresetAction::Store) {
          _nonVolatileStorage.storePreset(targetPresetIndex, &activeState);
          doneDisplayTimer = 0;
        }

        presetAction = PresetAction::None;
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

  calibrateDisplayTime += microsdelta;
  if (calibrateDisplayTime > OTHER_DISPLAY_TIME << 1) {
    calibrateDisplayTime = OTHER_DISPLAY_TIME;
  }
}

void Module::_display() {

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
    this->displayState = DisplayState::Pop;
  } else if (this->clockSwitch == LOW ||
             this->tempoDisplayTime < TEMPO_DISPLAY_TIME) {
    this->displayState = DisplayState::Tempo;
  } else if (this->inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::InputClockDivide;
    colon = true;
  } else if (this->beatModeDisplayTime < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::BeatMode;
  } else if (this->countdownDisplayTime < COUNTDOWN_DISPLAY_TIME && this->messd.modulationPending) {
    this->displayState = DisplayState::Countdown;
  } else if (this->beatsEqualsDivDisplayTime < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::BeatsEqualDivs;
  } else if (this->presetDisplayTimer < PRESET_DISPLAY_TIME) {
    this->displayState = DisplayState::Preset;
  } else if (this->calibrateDisplayTime < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::Calibration;
  } else if (this->doneDisplayTimer < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::Done;
  } else {
    this->displayState = DisplayState::Default;
    colon = true;
  }

  switch (digitCounter) {
  case 0:
    if (this->displayState == DisplayState::Tempo) {
      value = (displayableTempo) / 1000;
    } else if (this->displayState == DisplayState::Default) {
      value = activeDiv / 10;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatMode) {
      value =
          (int)(activeState.beatInputResetMode ? SpecialDigits::Nothing : SpecialDigits::B);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 1000;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::Preset) {
      value = (int)SpecialDigits::P;
    } else if (this->displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::C;
    } else if (this->displayState == DisplayState::Done) {
      value = (int)SpecialDigits::D;
    }
    break;
  case 1:
    if (this->displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 100) % 10;
    } else if (this->displayState == DisplayState::Default) {
      value = activeDiv % 10;
      decimal = this->outs.subdivision;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = (int)1;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (int)(activeState.beatInputResetMode ? SpecialDigits::R : SpecialDigits::E);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 100;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::D;
    } else if (this->displayState == DisplayState::Preset) {
      value = targetPresetIndex + 1;
    } else if (this->displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::A;
    } else if (this->displayState == DisplayState::Done) {
      value = 0;
    }
    break;
  case 2:
    if (this->displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 10) % 10;
      decimal = tempoDecimal;
    } else if (this->displayState == DisplayState::Default) {
      value = activeBeats / 10;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = activeState.inputClockDivider / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (activeState.beatInputResetMode ? 5 : (int)SpecialDigits::A);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Equals;
    } else if (this->displayState == DisplayState::Preset) {
      value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::Calibration) {
      value = (int)SpecialDigits::L;
    } else if (this->displayState == DisplayState::Done) {
      value = (int)SpecialDigits::N;
    }
    break;
  case 3:
    if (this->displayState == DisplayState::Tempo) {
      value = displayableTempo % 10;
    } else if (this->displayState == DisplayState::Default) {
      value = activeBeats % 10;
      decimal = this->outs.beat;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = activeState.inputClockDivider % 10;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (int)SpecialDigits::T;
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold % 10;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::B;
    } else if (this->displayState == DisplayState::Preset) {
      value = presetAction == PresetAction::None ?
        (int)SpecialDigits::Nothing :
        presetAction == PresetAction::Recall ?
        (int)SpecialDigits::R :
        5;
    } else if (this->displayState == DisplayState::Calibration) {
      value = 1;
    } else if (this->displayState == DisplayState::Done) {
      value = (int)SpecialDigits::E;
    }
    break;
  }

  hardware.sevenSegmentDisplay.process(digitCounter, value, decimal, colon);
}

void Module::HardwareRead(messd_ins_t *ins, messd_outs_t *outs){};
void Module::HardwareWrite(messd_ins_t *ins, messd_outs_t *outs){};

void Module::initHardware() {
  Serial.begin(9600);

  thresh_init(&beatSwitchThreshold, BEAT_INPUT_THRESH, BEAT_INPUT_HIST);

  // while (!Serial) {}
  // delay (100);

// Enable interrupts for the clock input pin
// TODO: rp2040
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
 {
  MS_init(&this->messd);
};

void Module::process(float microsDelta) {

  if (!_nonVolatileStorageInitialized) {
    // Load initial preset
    SerializableState initialData;
    uint8_t loaded = _nonVolatileStorage.loadLastPreset(&initialData);
    if (loaded) {
      memcpy(&activeState, &initialData, sizeof(SerializableState));
      tapTempoOut = activeState.tapTempo;
    }

    // Load calibration
    CalibratedState calibration;
    char buff[128];
    sprintf(buff, "/littlefs/calibration.txt");
    loaded = _nonVolatileStorage.read(buff, &calibration);
    if (loaded) {
      memcpy(&calibratedState, &calibration, sizeof(CalibratedState));
    }
    _beatCVQuantizer.setInMid(calibratedState.beatInputMid);
    _divCVQuantizer.setInMid(calibratedState.divInputMid);

    _nonVolatileStorageInitialized = true;
  }

  // Serial.println(calibratedState.beatInputMid);

  isClockInternal = !digitalRead(clockJackSwitch);
  uint8_t clockInput = isClockInternal ? lastInternalClockState : lastExternalClock;
  float ratio = (float) this->messd.tempoDivide / (float) this->messd.tempoMultiply;

  // Let's process the encoders as often as we can, basically after every big
  // operation
  hardware.analogMux.process();
  isRoundTripMode = hardware.analogMux.getOutput(AnalogMux.ROUND_SWITCH) < (MAX_VOLTAGE >> 1);
  _processCalibration(microsDelta);

  hardware.digitalMux.process();
  _processEncoders(ratio);

  _processTapTempo(microsDelta);
  _processModSwitch(microsDelta);
  _processBeatDivSwitches(microsDelta);

  // for (int i = 0; i < 8; i++) {
  //   if (i != 0) Serial.print(",\t");
  //   Serial.print(hardware.analogMux.getOutput(i));
  // }
  // Serial.println();

  // return;

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
  int divOffset = qdivInput * divAttenuvert;

  float divBase = activeState.subdivisions;
  activeDiv = min(beatsDivMax, max(beatsDivMin, divBase + divOffset));

  int beatsOffset = 0;
  int beatsBase = activeState.beats;
  if (!activeState.beatInputResetMode) {
    int beatSigInput = max(BEAT_INPUT_MIN, min(BEAT_INPUT_MAX, rawBeatInput));
    beatsOffset = _beatCVQuantizer.process(beatSigInput);
  }
  activeBeats = min(beatsDivMax, max(beatsDivMin, beatsBase + beatsOffset));

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
  truncationOffset *= 0.5; // right?

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
    this->beatsEqualsDivDisplayTime = 0;
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

  // about 100 msec

  // for (int i = 0; i < 8; i++) {
  //   if (i != 0) Serial.print(", ");
  //   Serial.print(output_sr_val[i]);
  // }
  // Serial.print("\n");
  hardware.moduleOuts.process(this->output_sr_val, 8, true);

  // Configure LEDs
  bool beatLatchDisplay = activeState.beat_latch;
  if (activeState.beat_latch && (activeBeats != this->messd.beatsPerMeasure)) {
    beatLatchDisplay = this->latchPulseTimer > (LATCH_PULSE_TIME / 2.0f);
  }
  bool divLatchDisplay = activeState.div_latch;
  if (activeState.div_latch && (activeDiv != this->messd.subdivisionsPerMeasure)) {
    divLatchDisplay = this->latchPulseTimer > (LATCH_PULSE_TIME / 2.0f);
  }
  leds_sr_val[(uint8_t)LEDNames::Nothing] = HIGH;
  leds_sr_val[(uint8_t)LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::EoMLED] =
      this->eomBuffer < EOM_LED_BUFFER_MICROS ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::ClockLEDButton] =
      clockInput == HIGH ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::DownbeatLED] =
      this->outs.downbeat ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::BeatLatchLED] = beatLatchDisplay ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::DivLatchLED] = divLatchDisplay ? LOW : HIGH;

  // about 100 msec
  // for (int i = 0; i < 8; i++) {
  //   if (i != 0) Serial.print(", ");
  //   Serial.print(leds_sr_val[i]);
  // }
  // Serial.println();
  hardware.moduleLEDs.process(this->leds_sr_val, 8, true);

  this->scaledTempo = this->outs.scaledTempo;

  // about 400 msec
  _display();

  hardware.digitalMux.process();
  _processEncoders(ratio);

  this->eomBuffer += microsDelta;
  if (this->eomBuffer > 5000000)
    this->eomBuffer = 5000000;
  this->latchPulseTimer += microsDelta;
  if (this->latchPulseTimer > LATCH_PULSE_TIME) {
    this->latchPulseTimer = fmod(this->latchPulseTimer, LATCH_PULSE_TIME);
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

  this->beatsEqualsDivDisplayTime += microsDelta;
  if (this->beatsEqualsDivDisplayTime > OTHER_DISPLAY_TIME + 1)
    this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME + 1;

  this->presetDisplayTimer += microsDelta;
  if (this->presetDisplayTimer > PRESET_DISPLAY_TIME * 2) {
    this->presetDisplayTimer = PRESET_DISPLAY_TIME;
  }

  this->doneDisplayTimer += microsDelta;
    if (this->doneDisplayTimer > OTHER_DISPLAY_TIME * 2) {
    this->doneDisplayTimer = OTHER_DISPLAY_TIME;
  }
};
