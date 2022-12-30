#include "messd-up.hpp"
#include "timers.hpp"
#include "voltages.hpp"

#include <avr/interrupt.h>

#if USING_MBED_RPI_PICO
  #include "MBED_RPi_Pico_TimerInterrupt.h"

  MBED_RPI_PICO_Timer ITimer0(0);
#else
  #define USE_TIMER_1 true
  #define USE_TIMER_2 true
  #include <TimerInterrupt.h>
#endif

// Got to be globals to work with the arduino timer. TODO: Figure out how to do
// this on other platforms Internal clock
int clockIn = CLK_IN_A;
int clockOut = CLK_IN_TN_A;

// More accurate input clock high detection
volatile uint8_t lastInternalClockState = LOW;
volatile unsigned long lastHighClockTime = 0;
uint16_t clockPhaseCounter = 0;
volatile uint8_t inputClockCounter = 0;
volatile uint8_t inputClockDivider = 1;
volatile uint8_t lastClock = LOW;

// Beat input reset
volatile uint8_t beatInputResetMode = 0;

// Tempo
volatile float tapTempoOut = 131.0;
float lastTapTempoOut = 131.0;

#define DIV_INPUT_CALIBRATION (-9)

static TIMER_HEADER(TimerHandler)
  clockPhaseCounter++;

  if (clockPhaseCounter >= 100) {
    clockPhaseCounter = 0;
    lastInternalClockState = !lastInternalClockState;
    digitalWrite(clockOut, lastInternalClockState);
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
      lastClock = HIGH;
    } else {
      lastClock = LOW;
    }

    inputClockCounter = (inputClockCounter + 1) % inputClockDivider;
  } else {
    lastClock = LOW;
  }
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

void Module::_processEncoders() {
  int inc = hardware.beat.process(
      hardware.digitalMux.getOutput(DigitalMux.BEAT_ENC_A),
      hardware.digitalMux.getOutput(DigitalMux.BEAT_ENC_B));
  if (inc != 0) {

    // When you hold down the clock button, you set the tempo with the knobs
    if (this->clockSwitch == LOW) {
      this->tapTempo += inc;
      tapTempoOut += inc;
      if (this->tapTempo < Module::tempoMin)
        this->tapTempo = tapTempoOut = Module::tempoMin;
      if (this->tapTempo > Module::tempoMax)
        this->tapTempo = tapTempoOut = Module::tempoMax;
    } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
      // No-op, the beat encoder doesn't do anything here
    } else {
      this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
      this->beatModeDisplayTime = OTHER_DISPLAY_TIME;
      this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME;

      // You can't change beats when you're in a round trip modulation, in latch
      // mode
      if (!(messd.inRoundTripModulation && ins.latchModulationToDownbeat)) {
        state.beats += inc;
        this->latchPulseTimer = 0.0f;
        if (state.beats < beatsDivMin)
          state.beats = beatsDivMax;
        if (state.beats > beatsDivMax)
          state.beats = beatsDivMin;
      }
    }
  }

  inc = hardware.div.process(
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_A),
      hardware.digitalMux.getOutput(DigitalMux.DIVIDE_ENC_B));

  if (inc != 0) {
    // When you hold down the clock button, you set the tempo with the knobs
    if (this->clockSwitch == LOW) {
      this->tapTempo += inc * 10;
      tapTempoOut += inc * 10;
      if (this->tapTempo < Module::tempoMin)
        this->tapTempo = tapTempoOut = Module::tempoMin;
      if (this->tapTempo > Module::tempoMax)
        this->tapTempo = tapTempoOut = Module::tempoMax;
    } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
      inputClockDivider += inc;
      this->inputClockDivDisplayTime = 0.0f;
      if (inputClockDivider < Module::inputClockDivideMin)
        inputClockDivider = Module::inputClockDivideMin;
      if (inputClockDivider > Module::inputClockDivideMax)
        inputClockDivider = Module::inputClockDivideMax;
    } else {
      this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
      this->beatModeDisplayTime = OTHER_DISPLAY_TIME;
      this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME;

      state.div += inc;
      this->latchPulseTimer = 0.0f;
      if (state.div < beatsDivMin)
        state.div = beatsDivMax;
      if (state.div > beatsDivMax)
        state.div = beatsDivMin;
    }
  }
}

void Module::_processTapTempo(float msDelta) {
  int nextClockSwitch = hardware.digitalMux.getOutput(DigitalMux.CLOCK_SWITCH);

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
      this->tapTempo = 60000000.0 / ((double)delta);
      totalTaps++;
    } else {
      this->tapTempo =
          (60000000.0 / ((double)delta)) * (1.0 / (double)totalTaps) +
          this->tapTempo * ((totalTaps - 1) / (double)totalTaps);
      tapTempoOut = this->tapTempo;
      totalTaps = totalTaps >= 5 ? 5 : (totalTaps + 1);
    }

    this->lastTapMicros = nextTapMicros;

    if (this->tapTempo < Module::tapTempoMin)
      this->tapTempo = tapTempoOut = Module::tapTempoMin;
    if (this->tapTempo > Module::tapTempoMax)
      this->tapTempo = tapTempoOut = Module::tapTempoMax;
  }

  if (nextClockSwitch != LOW) {
    this->tempoDisplayTime += msDelta;
    if (this->tempoDisplayTime > TEMPO_DISPLAY_TIME * 2) {
      this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
    }
  } else {
    this->tempoDisplayTime = 0;
    this->inputClockDivDisplayTime = OTHER_DISPLAY_TIME;
  }

  this->clockSwitch = nextClockSwitch;
}

void Module::_processModSwitch(float msDelta) {
  /* hardware.MODSWITCH.PinRead(); */
  hardware.MODSWITCH.val = digitalRead(hardware.MODSWITCH.address);

  if (hardware.MODSWITCH.val == LOW) {
    this->modHoldTime += msDelta;
  } else {
    this->modHoldTime = 0;
    this->canTriggerReset = true;
  }
}

void Module::_processBeatDivSwitches(float msDelta) {
  // div switch
  if (hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH) == LOW) {
    if (div_switch_state_prev == HIGH) {
      initial_div_latch = div_latch;
      div_latch = !div_latch;
    } else {
      divHoldTime += msDelta;
      if (divHoldTime >= DIV_BUTTON_HOLD_TIME) {
        div_latch = initial_div_latch;
        inputClockDivDisplayTime = 0.0f;
      }
    }
  } else {
    inputClockDivDisplayTime += msDelta;
    if (inputClockDivDisplayTime > OTHER_DISPLAY_TIME + 1)
      inputClockDivDisplayTime = OTHER_DISPLAY_TIME + 1;
    divHoldTime = 0.0f;
  }
  div_switch_state_prev = hardware.digitalMux.getOutput(DigitalMux.DIV_SWITCH);

  // beat switch
  if (hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH) == LOW) {
    if (beat_switch_state_prev == HIGH) {
      initial_beat_latch = beat_latch;
      beat_latch = !beat_latch;
    } else {
      beatHoldTime += msDelta;
      if (canSwitchBeatInputModes && beatHoldTime > BEAT_BUTTON_HOLD_TIME) {
        beat_latch = initial_beat_latch;
        beatInputResetMode = !beatInputResetMode;
        canSwitchBeatInputModes = false;
        beatModeDisplayTime = 0.0f;
      }
    }
  } else {
    beatModeDisplayTime += msDelta;
    if (beatModeDisplayTime > OTHER_DISPLAY_TIME + 1)
      beatModeDisplayTime = OTHER_DISPLAY_TIME + 1;
    beatHoldTime = 0.0f;
    canSwitchBeatInputModes = true;
  }
  beat_switch_state_prev =
      hardware.digitalMux.getOutput(DigitalMux.BEAT_SWITCH);
}

void Module::_display() {

  digitCounter++;
  if (digitCounter == 4) {
    digitCounter = 0;
  }

  int value, decimal = 0, colon = 0;
  int tempoDecimal = 1;
  long displayableTempo = (long)round(this->scaledTempo * 10.0f);
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
  } else if (this->countdownDisplayTime < COUNTDOWN_DISPLAY_TIME) {
    this->displayState = DisplayState::Countdown;
  } else if (this->beatsEqualsDivDisplayTime < OTHER_DISPLAY_TIME) {
    this->displayState = DisplayState::BeatsEqualDivs;
  } else {
    this->displayState = DisplayState::Default;
    colon = true;
  }

  switch (digitCounter) {
  case 0:
    if (this->displayState == DisplayState::Tempo) {
      value = (displayableTempo) / 1000;
    } else if (this->displayState == DisplayState::Default) {
      value = state.activeDiv / 10;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatMode) {
      value =
          (int)(beatInputResetMode ? SpecialDigits::Nothing : SpecialDigits::B);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 1000;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Nothing;
    }
    break;
  case 1:
    if (this->displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 100) % 10;
    } else if (this->displayState == DisplayState::Default) {
      value = state.activeDiv % 10;
      decimal = this->outs.subdivision;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = (int)1;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (int)(beatInputResetMode ? SpecialDigits::R : SpecialDigits::E);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 100;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::D;
    }
    break;
  case 2:
    if (this->displayState == DisplayState::Tempo) {
      value = ((displayableTempo) / 10) % 10;
      decimal = tempoDecimal;
    } else if (this->displayState == DisplayState::Default) {
      value = state.activeBeats / 10;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = inputClockDivider / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (beatInputResetMode ? 5 : (int)SpecialDigits::A);
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold / 10;
      if (value == 0)
        value = (int)SpecialDigits::Nothing;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::Equals;
    }
    break;
  case 3:
    if (this->displayState == DisplayState::Tempo) {
      value = displayableTempo % 10;
    } else if (this->displayState == DisplayState::Default) {
      value = state.activeBeats % 10;
      decimal = this->outs.beat;
    } else if (this->displayState == DisplayState::Pop) {
      value = (int)SpecialDigits::Dash;
    } else if (this->displayState == DisplayState::InputClockDivide) {
      value = inputClockDivider % 10;
    } else if (this->displayState == DisplayState::BeatMode) {
      value = (int)SpecialDigits::T;
    } else if (this->displayState == DisplayState::Countdown) {
      value = countdownSampleAndHold % 10;
    } else if (this->displayState == DisplayState::BeatsEqualDivs) {
      value = (int)SpecialDigits::B;
    }
    break;
  }

  hardware.sevenSegmentDisplay.process(digitCounter, value, decimal, colon);
}

void Module::HardwareRead(messd_ins_t *ins, messd_outs_t *outs){};
void Module::HardwareWrite(messd_ins_t *ins, messd_outs_t *outs){};

void Module::initHardware() {

  Serial.begin(9600);

  while (!Serial) {}
  delay (100);

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

  this->eomBuffer = EOM_BUFFER_MS;

  // Analog read pin for the digital mux
  /* hardware.DIGITAL_MUX_IN.PinInit(); */

  // Digital read for the dedicated mod switch pin
  /* hardware.MODSWITCH.PinInit(); */
  pinMode(hardware.MODSWITCH.address, hardware.MODSWITCH.type);

  // Clock pins
  // TODO - make part of Hardware class (Does it really have to be global?)
  pinMode(clockIn, INPUT);
  pinMode(clockOut, OUTPUT);

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

Module::Module() { MS_init(&this->messd); };

void Module::process(float msDelta) {

  uint8_t clockInput = lastClock;

  // Let's process the encoders as often as we can, basically after every big
  // operation
  hardware.digitalMux.process();

  _processEncoders();

  hardware.analogMux.process();

  _processTapTempo(msDelta);
  _processModSwitch(msDelta);
  _processBeatDivSwitches(msDelta);

  // for (int i = 0; i < 8; i++) {
  //   if (i != 0) Serial.print(",\t");
  //   Serial.print(hardware.analogMux.getOutput(i));
  // }
  // Serial.println();

  static int beatInputMid = (BEAT_INPUT_MAX + BEAT_INPUT_MIN) / 2;
  uint8_t beatInput =
      hardware.analogMux.getOutput(AnalogMux.BEAT_INPUT) < beatInputMid ? 1 : 0;
  uint8_t didReset = 0;
  this->ins.resetBeatCount = 0;
  if (beatInputResetMode && (beatInput && !this->lastBeatInputValue)) {
    this->ins.resetBeatCount = 1;
    didReset = 1;
    Serial.println("reset");
  }
  this->lastBeatInputValue = beatInput;

  // Compute the final value for subdivisions and beats based on modulation
  // inputs and attenuverters
  static float divInputRange = (DIV_INPUT_MAX - DIV_INPUT_MIN);
  int divInput = hardware.analogMux.getOutput(AnalogMux.DIVIDE_INPUT);
  divInput = max(DIV_INPUT_MIN, min(DIV_INPUT_MAX, divInput));
  float divOffset = 1.0f - (float)(divInput - DIV_INPUT_MIN) / (float)divInputRange;

  static float divAttenuvertRange = (DIV_ATV_MAX - DIV_ATV_MIN);
  int divAttenuvertInput = hardware.analogMux.getOutput(AnalogMux.DIVIDE_ATV);
  divAttenuvertInput = max(DIV_ATV_MIN, min(DIV_ATV_MAX, divInput));
  float divAttenuvert =
      (float)(divAttenuvertInput - DIV_ATV_MIN) / divAttenuvertRange;
  divAttenuvert = 2.0f * (divAttenuvert - 0.5);
  float divBase = (float)(this->state.div - beatsDivMin) /
                  (float)(beatsDivMax - beatsDivMin);
  float finalDivNormalized =
      fmax(0.0, fmin(1.0, divBase + divOffset * divAttenuvert));
  state.activeDiv =
      round(finalDivNormalized * (beatsDivMax - beatsDivMin)) + beatsDivMin;

  static int beatInputRange = (BEAT_INPUT_MAX - BEAT_INPUT_MIN);
  static int beatInputGrain = beatInputRange / (beatsDivMax - beatsDivMin);
  int beatSigInput = hardware.analogMux.getOutput(AnalogMux.BEAT_INPUT);
  beatSigInput = max(BEAT_INPUT_MIN, min(BEAT_INPUT_MAX, beatSigInput));
  beatSigInput -= beatInputMid;
  int beatsOffset = beatSigInput / beatInputGrain;
  int beatsBase = this->state.beats;
  state.activeBeats = min(beatsDivMax, max(beatsDivMin, beatsBase + beatsOffset));

  this->ins.tempo = tapTempoOut;
  this->ins.beatsPerMeasure = state.activeBeats;
  this->ins.subdivisionsPerMeasure = state.activeDiv;
  this->ins.phase = 0; // unused
  this->ins.ext_clock = clockInput == HIGH;

  static int modInputMid = (MOD_INPUT_MAX + MOD_INPUT_MIN) / 2;
  this->ins.modulationSignal =
      hardware.analogMux.getOutput(AnalogMux.MOD_INPUT) > modInputMid ? 1 : 0;
  this->ins.modulationSwitch = this->modSwitch == LOW; // active low
  this->ins.latchBeatChangesToDownbeat = beat_latch;
  this->ins.latchDivChangesToDownbeat = div_latch;

  // TODO: Get this working after Max solders on the resistors
  this->ins.latchModulationToDownbeat =
      hardware.analogMux.getOutput(AnalogMux.LATCH_SWITCH) > (MAX_VOLTAGE >> 1);
  this->ins.invert = 0; // unused
  this->ins.isRoundTrip =
      hardware.analogMux.getOutput(AnalogMux.ROUND_SWITCH) < (MAX_VOLTAGE >> 1);

  // Temporary: Set the values of the switches explicitly, until we figure
  // out what's going on with the hardware.
  this->ins.latchModulationToDownbeat = 1;
  this->ins.isRoundTrip = 1;

  if (this->modHoldTime > MOD_BUTTON_RESET_TIME_MS && this->canTriggerReset) {
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
  truncationInput = min(TRUNC_ATV_MAX, max(TRUNC_ATV_MIN, truncationInput));
  float truncationOffset =
      1.0f - (float)(truncationInput - TRUNC_ATV_MIN) /
                 (float)truncInputRange;

#ifndef IS_POWERED_FROM_ARDUINO
  truncationOffset -= 0.5f;
  truncationOffset *= 2.0f;
#endif
  baseTruncation = fmax(0.0, fmin(1.0, baseTruncation + truncationOffset));
  this->ins.truncation = baseTruncation >= 1.0f ? 1.0f : baseTruncation;

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

  this->ins.cheatedMeasuredPeriod = measuredPeriod;

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
  _processEncoders();

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
  this->countdownDisplayTime += msDelta;
  this->countdownDisplayTime =
      min(this->countdownDisplayTime, COUNTDOWN_DISPLAY_TIME);
  this->lastDownbeat = this->outs.downbeat;

  if (this->outs.eom) {
    this->eomBuffer = 0;
    this->animateModulateButtonTime = 0.0f;
    this->modButtonFlashTimer = 0.0;
    this->modButtonFlashCount = 0;

    this->state.div = this->ins.subdivisionsPerMeasure;
    this->state.beats = this->ins.beatsPerMeasure;

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
    this->animateModulateButtonTime += msDelta;
    if (this->animateModulateButtonTime > (2.0f * MOD_BUTTON_STROBE_SLOW)) {
      this->animateModulateButtonTime = fmodf(this->animateModulateButtonTime,
                                              (2.0f * MOD_BUTTON_STROBE_SLOW));
    }
  } else if (this->outs.inRoundTripModulation) {
    modButtonOn = true;
  }

  hardware.digitalMux.process();
  _processEncoders();

  // Configure outputs
  output_sr_val[(uint8_t)OutputNames::Nothing] = HIGH;
  output_sr_val[(uint8_t)OutputNames::TruncateLED] =
      this->outs.truncate ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::DivLED] =
      this->outs.subdivision ? LOW : HIGH;
  output_sr_val[(uint8_t)OutputNames::EoMOutput] =
      this->eomBuffer < EOM_BUFFER_MS ? LOW : HIGH;
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
  bool beatLatchDisplay = this->beat_latch;
  if (this->state.activeBeats != this->messd.beatsPerMeasure) {
    beatLatchDisplay = this->latchPulseTimer > (LATCH_PULSE_TIME / 2.0f);
  }
  bool divLatchDisplay = this->div_latch;
  if (this->state.activeDiv != this->messd.subdivisionsPerMeasure) {
    divLatchDisplay = this->latchPulseTimer > (LATCH_PULSE_TIME / 2.0f);
  }
  leds_sr_val[(uint8_t)LEDNames::Nothing] = HIGH;
  leds_sr_val[(uint8_t)LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
  leds_sr_val[(uint8_t)LEDNames::EoMLED] =
      this->eomBuffer < EOM_LED_BUFFER_MS ? LOW : HIGH;
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
  _processEncoders();

  this->eomBuffer += msDelta;
  if (this->eomBuffer > 5000000)
    this->eomBuffer = 5000000;
  this->latchPulseTimer += msDelta;
  if (this->latchPulseTimer > LATCH_PULSE_TIME) {
    this->latchPulseTimer = fmod(this->latchPulseTimer, LATCH_PULSE_TIME);
  }

  if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
    this->modButtonFlashTimer += msDelta;
    if (this->modButtonFlashTimer > MOD_BUTTON_FLASH_TIME) {
      this->modButtonFlashCount++;
      this->modButtonFlashTimer =
          (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT)
              ? 0.0
              : MOD_BUTTON_FLASH_TIME;
    }
  }

  this->beatsEqualsDivDisplayTime += msDelta;
  if (this->beatsEqualsDivDisplayTime > OTHER_DISPLAY_TIME + 1)
    this->beatsEqualsDivDisplayTime = OTHER_DISPLAY_TIME + 1;
};
