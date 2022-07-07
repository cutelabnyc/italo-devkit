#include "messd-up.hpp"
#include <avr/interrupt.h>

#define USE_TIMER_1     true
#define USE_TIMER_2     true
#include <TimerInterrupt.h>

// Got to be globals to work with the arduino timer. TODO: Figure out how to do this on other platforms
// Internal clock
int clockIn = 12; // PB4
int clockOut = A1; // PC1

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

static void TimerHandler()
{
    clockPhaseCounter++;

    if (clockPhaseCounter >= 100) {
        clockPhaseCounter = 0;
        lastInternalClockState = !lastInternalClockState;
        digitalWrite(clockOut, lastInternalClockState);
    }

    if (lastTapTempoOut != tapTempoOut) {
        float timerFreqHz = tapTempoOut * 100.0f / 30.0f; // double speed because it alternates at 2z Hz
        lastTapTempoOut = tapTempoOut;
        ITimer1.setFrequency(timerFreqHz, TimerHandler);
    }
}

ISR (PCINT0_vect) {
    if (PINB & 0b00010000) {
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
    this->ins.subdivisionsPerMeasure = (this->ins.subdivisionsPerMeasure > 0
        ? this->ins.subdivisionsPerMeasure
        : 1);
    this->ins.phase =
        (this->ins.phase <= 1 && ins.phase >= 0 ? this->ins.phase : 0);
    this->ins.pulseWidth = (this->ins.pulseWidth < 1 && this->ins.pulseWidth > 0
        ? this->ins.pulseWidth
        : 0.5);
}

void Module::_processEncoders() {
    int inc =
        encoder_process(&beat_encoder, digital_mux.outputs[DigitalMux.BEAT_ENC_A],
            digital_mux.outputs[DigitalMux.BEAT_ENC_B]);
    if (inc != 0) {

        // When you hold down the clock button, you set the tempo with the knobs
        if (this->clockSwitch == LOW) {
            this->tapTempo += inc;
            tapTempoOut += inc;
            if (this->tapTempo < Module::tempoMin) this->tapTempo = tapTempoOut = Module::tempoMin;
            if (this->tapTempo > Module::tempoMax) this->tapTempo = tapTempoOut = Module::tempoMax;
        } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
            // No-op, the beat encoder doesn't do anything here
        } else {
            this->tempoDisplayTime = TEMPO_DISPLAY_TIME;

            // You can't change beats when you're in a round trip modulation, in latch mode
            if (!(messd.inRoundTripModulation && ins.latchModulationToDownbeat)) {
                state.beats += inc;
                if (state.beats < beatsDivMin)
                    state.beats = beatsDivMax;
                if (state.beats > beatsDivMax)
                    state.beats = beatsDivMin;
            }
        }
    }
    inc = encoder_process(&div_encoder,
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_A],
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_B]);
    if (inc != 0) {
        // When you hold down the clock button, you set the tempo with the knobs
        if (this->clockSwitch == LOW) {
            this->tapTempo += inc * 10;
            tapTempoOut += inc * 10;
            if (this->tapTempo < Module::tempoMin) this->tapTempo = tapTempoOut = Module::tempoMin;
            if (this->tapTempo > Module::tempoMax) this->tapTempo = tapTempoOut = Module::tempoMax;
        } else if (inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
            inputClockDivider += inc;
            this->inputClockDivDisplayTime = 0.0f;
            if (inputClockDivider < Module::inputClockDivideMin) inputClockDivider = Module::inputClockDivideMin;
            if (inputClockDivider > Module::inputClockDivideMax) inputClockDivider = Module::inputClockDivideMax;
        } else {
            this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
            state.div += inc;

            if (state.div < beatsDivMin)
                state.div = beatsDivMax;
            if (state.div > beatsDivMax)
                state.div = beatsDivMin;
        }
    }
}

void Module::_processTapTempo(float msDelta) {
    int nextClockSwitch = digital_mux.outputs[DigitalMux.CLOCK_SWITCH];

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
            this->tapTempo = 60000000.0 / ((double) delta);
            totalTaps++;
        } else {
            this->tapTempo = (60000000.0 / ((double) delta)) * (1.0 / (double) totalTaps) + this->tapTempo * ((totalTaps - 1) / (double) totalTaps);
            tapTempoOut = this->tapTempo;
            totalTaps = totalTaps >= 5 ? 5 : (totalTaps + 1);
        }

        this->lastTapMicros = nextTapMicros;

        if (this->tapTempo < Module::tapTempoMin) this->tapTempo = tapTempoOut = Module::tapTempoMin;
        if (this->tapTempo > Module::tapTempoMax) this->tapTempo = tapTempoOut = Module::tapTempoMax;
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
    this->modSwitch = digitalRead(modSwitchPin);

    if (this->modSwitch == LOW) {
        this->modHoldTime += msDelta;
    } else {
        this->modHoldTime = 0;
        this->canTriggerReset = true;
    }
}

void Module::_processBeatDivSwitches(float msDelta) {
    // div switch
    if (digital_mux.outputs[DigitalMux.DIV_SWITCH] == LOW) {
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
    div_switch_state_prev = digital_mux.outputs[DigitalMux.DIV_SWITCH];

    // beat switch
    if (digital_mux.outputs[DigitalMux.BEAT_SWITCH] == LOW) {
        if (beat_switch_state_prev == HIGH) {
            initial_beat_latch = beat_latch;
            beat_latch = !beat_latch;
        } else {
            beatHoldTime += msDelta;
            if (canSwtichBeatInputModes && beatHoldTime > BEAT_BUTTON_HOLD_TIME) {
                beat_latch = initial_beat_latch;
                beatInputResetMode = !beatInputResetMode;
                canSwtichBeatInputModes = false;
                beatModeDisplayTime = 0.0f;
            }
        }
    } else {
        beatModeDisplayTime += msDelta;
        if (beatModeDisplayTime > OTHER_DISPLAY_TIME + 1)
            beatModeDisplayTime = OTHER_DISPLAY_TIME + 1;
        beatHoldTime = 0.0f;
        canSwtichBeatInputModes = true;
    }
    beat_switch_state_prev = digital_mux.outputs[DigitalMux.BEAT_SWITCH];
}

void Module::_display() {

    digitCounter++;
    if (digitCounter == 4) {
        digitCounter = 0;
    }

    int value, decimal = 0, colon = 0;
    long displayableTempo = (long) round(this->scaledTempo * 10.0f);

    if (this->outs.resetPending) {
        this->displayState = DisplayState::Pop;
    } else if (this->clockSwitch == LOW || this->tempoDisplayTime < TEMPO_DISPLAY_TIME) {
        this->displayState = DisplayState::Tempo;
    } else if (this->inputClockDivDisplayTime < OTHER_DISPLAY_TIME) {
        this->displayState = DisplayState::InputClockDivide;
        colon = true;
    } else if (this->beatModeDisplayTime < OTHER_DISPLAY_TIME) {
        this->displayState = DisplayState::BeatMode;
    } else if (this->countdownDisplayTime < COUNTDOWN_DISPLAY_TIME) {
        this->displayState = DisplayState::Countdown;
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
                value = (int) SpecialDigits::Dash;
            } else if (this->displayState == DisplayState::InputClockDivide) {
                value = (int) SpecialDigits::Nothing;
            } else if (this->displayState == DisplayState::BeatMode) {
                value = (int) (beatInputResetMode ? SpecialDigits::Nothing : SpecialDigits::B);
            } else if (this->displayState == DisplayState::Countdown) {
                value = countdownSampleAndHold / 1000;
                if (value == 0) value = (int) SpecialDigits::Nothing;
            }
            break;
        case 1:
            if (this->displayState == DisplayState::Tempo) {
                value = ((displayableTempo) / 100) % 10;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeDiv % 10;
				decimal = this->outs.subdivision;
            } else if (this->displayState == DisplayState::Pop) {
                value = (int) SpecialDigits::Dash;
            } else if (this->displayState == DisplayState::InputClockDivide) {
                value = (int) 1;
            } else if (this->displayState == DisplayState::BeatMode) {
                value = (int) (beatInputResetMode ? SpecialDigits::R : SpecialDigits::E);
            } else if (this->displayState == DisplayState::Countdown) {
                value = countdownSampleAndHold / 100;
                if (value == 0) value = (int) SpecialDigits::Nothing;
            }
            break;
        case 2:
            if (this->displayState == DisplayState::Tempo) {
                value = ((displayableTempo) / 10) % 10;
                decimal = 1;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeBeats / 10;
            } else if (this->displayState == DisplayState::Pop) {
                value = (int) SpecialDigits::Dash;
            } else if (this->displayState == DisplayState::InputClockDivide) {
                value = inputClockDivider / 10;
                if (value == 0) value = (int) SpecialDigits::Nothing;
            } else if (this->displayState == DisplayState::BeatMode) {
                value = (beatInputResetMode ? 5 : (int) SpecialDigits::A);
            } else if (this->displayState == DisplayState::Countdown) {
                value = countdownSampleAndHold / 10;
                if (value == 0) value = (int) SpecialDigits::Nothing;
            }
            break;
        case 3:
            if (this->displayState == DisplayState::Tempo) {
                value = displayableTempo % 10;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeBeats % 10;
				decimal = this->outs.beat;
            } else if (this->displayState == DisplayState::Pop) {
                value = (int) SpecialDigits::Dash;
            } else if (this->displayState == DisplayState::InputClockDivide) {
                value = inputClockDivider % 10;
            } else if (this->displayState == DisplayState::BeatMode) {
                value = (int) SpecialDigits::T;
            } else if (this->displayState == DisplayState::Countdown) {
                value = countdownSampleAndHold % 10;
            }
            break;
    }

    seven_segment_process(&seven_segment_sr, digitCounter, value, decimal, colon);
}

Module::Module() {
    MS_init(&this->messd);
};

void Module::init() {

    // Enable interrupts for the clock input pin
    cli();
    PCICR |= 0b00000011; // Enables Port B Pin Change Interrupts
    PCMSK0 |= 0b00010000; // PB4
    sei();

    this->eomBuffer = EOM_BUFFER_MS;

    // Analog read pin for the digital mux
    pinMode(digitalMuxIn, INPUT);

    // Digital read for the dedicated mod switch pin
    pinMode(modSwitchPin, INPUT);

    // Setting all of the shift register pins to be outputs
    pinMode(clockPinSR, OUTPUT);
    pinMode(latchPinSR, OUTPUT);
    pinMode(dataPinSR, OUTPUT);
    pinMode(clockPinOuts, OUTPUT);
    pinMode(latchPinOuts, OUTPUT);
    pinMode(dataPinOuts, OUTPUT);
    pinMode(clockPinLEDs, OUTPUT);
    pinMode(latchPinLEDs, OUTPUT);
    pinMode(dataPinLEDs, OUTPUT);

    // Mux selector pins are outputs
    pinMode(m0, OUTPUT);
    pinMode(m1, OUTPUT);
    pinMode(m2, OUTPUT);

    // Clock pins
    pinMode(clockIn, INPUT);
    pinMode(clockOut, OUTPUT);

    for (int i = 0; i < 8; i++) {
        output_sr_val[i] = 0;
        leds_sr_val[i] = 0;
    }

    // Start the timer for the accurate clock
    float timerFreqHz = tapTempoOut * 100.0f / 30.0f; // double speed because it alternates at 2z Hz

    ITimer1.init();
    ITimer2.init();
    ITimer1.setFrequency(timerFreqHz, TimerHandler);
}

void Module::process(float msDelta) {

    uint8_t clockInput = lastClock;

    // Let's process the encoders as often as we can, basically after every big operation
    mux_process(&digital_mux);
    _processEncoders();

    mux_process(&analog_mux);

    _processTapTempo(msDelta);
    _processModSwitch(msDelta);
    _processBeatDivSwitches(msDelta);

    // Remember to change this back when the beat input is working
    // Serial.println(this->analog_mux.outputs[AnalogMux.TRUNCATE_INPUT]);
    uint8_t beatInput = this->analog_mux.outputs[AnalogMux.TRUNCATE_INPUT] < 475 ? 1 : 0;
    uint8_t didReset = 0;
    this->ins.resetBeatCount = 0;
    if (beatInputResetMode && (beatInput && !this->lastBeatInputValue)) {
        this->ins.resetBeatCount = 1;
        didReset = 1;
        Serial.println("reset");
    }
    this->lastBeatInputValue = beatInput;

    // Compute the final value for subdivisions and beats based on modulation inputs
    // and attenuverters
    float divOffset = 1.0f - (float) (this->analog_mux.outputs[AnalogMux.DIVIDE_INPUT] + DIV_INPUT_CALIBRATION) / (float) MAX_VOLTAGE;
#ifndef IS_POWERED_FROM_ARDUINO
    divOffset -= 0.5f;
    divOffset *= 2.0f;
#endif

    float divAttenuvert = (float) this->analog_mux.outputs[AnalogMux.DIVIDE_ATV] / (float) MAX_VOLTAGE;
    divAttenuvert = 2.0f * (divAttenuvert - 0.5);
    float divBase = (float) (this->state.div - beatsDivMin) / (float) (beatsDivMax - beatsDivMin);
    float finalDivNormalized = fmax(0.0, fmin(1.0, divBase + divOffset * divAttenuvert));
    state.activeDiv = round(finalDivNormalized * (beatsDivMax - beatsDivMin)) + beatsDivMin;

    float beatsOffset = (float) this->analog_mux.outputs[AnalogMux.BEAT_INPUT] / (float) MAX_VOLTAGE;
#ifndef IS_POWERED_FROM_ARDUINO
    beatsOffset -= 0.5f;
    beatsOffset *= 2.0f;
#endif
    beatsOffset = 0.0; // debug
    float beatsBase = (float) (this->state.beats - beatsDivMin) / (float) (beatsDivMax - beatsDivMin);
    float finalBeatsNormalized = fmax(0.0, fmin(1.0, beatsBase + beatsOffset));
    state.activeBeats = round(finalBeatsNormalized * (beatsDivMax - beatsDivMin)) + beatsDivMin;

    this->ins.tempo = tapTempoOut;
    this->ins.beatsPerMeasure = state.activeBeats;
    this->ins.subdivisionsPerMeasure = state.activeDiv;
    this->ins.phase = 0; // unused
    this->ins.ext_clock = clockInput == HIGH;
    this->ins.modulationSignal = this->analog_mux.outputs[AnalogMux.MOD_INPUT] > (MAX_VOLTAGE >> 1);
    this->ins.modulationSwitch = this->modSwitch == LOW; // active low
    this->ins.latchBeatChangesToDownbeat = beat_latch;
    this->ins.latchDivChangesToDownbeat = div_latch;
    this->ins.latchModulationToDownbeat = analog_mux.outputs[AnalogMux.LATCH_SWITCH] > (MAX_VOLTAGE >> 1);
    this->ins.invert = 0; // unused
    this->ins.isRoundTrip = analog_mux.outputs[AnalogMux.ROUND_SWITCH] < (MAX_VOLTAGE >> 1);

    if (this->modHoldTime > MOD_BUTTON_RESET_TIME_MS && this->canTriggerReset) {
        this->ins.reset = true;
        this->canTriggerReset = false;
    } else {
        this->ins.reset = false;
    }

    // compute wrap
    float baseTruncation = (float) this->analog_mux.outputs[AnalogMux.TRUNCATE_ATV] / (float) MAX_VOLTAGE;
    float truncationOffset = 1.0f - (float) this->analog_mux.outputs[AnalogMux.TRUNCATE_INPUT] / (float) MAX_VOLTAGE;

    // Remember to remove this when the beat input is working again
    if (beatInputResetMode) {
        truncationOffset = 0.5; // disable the truncation functionality of this input when in beat reset mode
    }

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
                measuredPeriod = (4294967295 - this->lastRecordedHighClockTime) + lastHighClockTime; // wraparound
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
    this->ins.delta = ((float) offset) / 1000.0;

    mux_process(&digital_mux);
    _processEncoders();

    MS_process(&this->messd, &this->ins, &this->outs);

    // static int lastModulationPending = false;
    // if (!lastModulationPending && this->messd.modulationPending) {

    //     uint8_t originalBeatsPerMeasure = messd.originalBeatsPerMeasure == 0 ? messd.beatsPerMeasure : messd.originalBeatsPerMeasure;
    //     float originalMeasurePhase = (messd.originalBeatCounter + messd.rootClockPhase) / originalBeatsPerMeasure;
    //     float scaledMeasurePhase = (messd.scaledBeatCounter + messd.scaledClockPhase) / messd.beatsPerMeasure;
    //     float scaleFactor = (1.0f - scaledMeasurePhase) / (1.0f - originalMeasurePhase);
    //     scaleFactor *= ((float) messd.tempoDivide) / ((float) messd.tempoMultiply);

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
    this->countdownDisplayTime = min(this->countdownDisplayTime, COUNTDOWN_DISPLAY_TIME);
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
        // float offsetRootMeasurePhase = (this->messd.rootClockPhase + this->messd.rootBeatCounter + this->messd.rootClockPhaseOffset);
        // if (offsetRootMeasurePhase > this->messd.tempoDivide) offsetRootMeasurePhase -= this->messd.tempoDivide;

        // float nextScaledClockPhase = (offsetRootMeasurePhase * this->messd.tempoMultiply) / this->messd.tempoDivide;
        // nextScaledClockPhase = fmod(nextScaledClockPhase, 1.0f);
        // Serial.println(nextScaledClockPhase);
        // Serial.println("---");
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
        modButtonOn = true;
	} else if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
		// Serial.println(modButtonFlashCount);
		modButtonOn = this->modButtonFlashCount % 2 > 0;
    } else if (this->outs.modulationPending) {
        modButtonOn = this->animateModulateButtonTime < MOD_BUTTON_STROBE_SLOW;
        this->animateModulateButtonTime += msDelta;
        if (this->animateModulateButtonTime > (2.0f * MOD_BUTTON_STROBE_SLOW)) {
            this->animateModulateButtonTime = fmodf(this->animateModulateButtonTime, (2.0f * MOD_BUTTON_STROBE_SLOW));
        }
    } else if (this->outs.inRoundTripModulation) {
        modButtonOn = true;
    }

    mux_process(&digital_mux);
    _processEncoders();

    // Configure outputs
    output_sr_val[(uint8_t) OutputNames::Nothing] = HIGH;
    output_sr_val[(uint8_t) OutputNames::TruncateLED] = this->outs.truncate ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::DivLED] = this->outs.subdivision ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::EoMOutput] = this->eomBuffer < EOM_BUFFER_MS ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::TruncateOutput] = this->outs.truncate ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::DivOutput] = this->outs.subdivision ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::DownbeatOutput] = this->outs.downbeat ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::BeatOutput] = this->outs.beat ? LOW : HIGH;
    // about 100 msec
    shift_register_process(&output_sr, this->output_sr_val, 8, true);

    // Configure LEDs
	bool beatLatchDisplay = this->beat_latch;
	if (this->state.activeBeats != this->messd.beatsPerMeasure) {
		beatLatchDisplay = this->latchPulseTimer < (LATCH_PULSE_TIME / 2.0f);
	}
	bool divLatchDisplay = this->div_latch;
	if (this->state.activeDiv != this->messd.subdivisionsPerMeasure) {
		divLatchDisplay = this->latchPulseTimer < (LATCH_PULSE_TIME / 2.0f);
	}
    leds_sr_val[(uint8_t) LEDNames::Nothing] = HIGH;
    leds_sr_val[(uint8_t) LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::EoMLED] = this->eomBuffer < EOM_LED_BUFFER_MS ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::ClockLEDButton] = clockInput == HIGH ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::DownbeatLED] = this->outs.downbeat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLatchLED] = beatLatchDisplay ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::DivLatchLED] = divLatchDisplay ? LOW : HIGH;

    // about 100 msec
    shift_register_process(&this->leds_sr, this->leds_sr_val, 8, true);

    this->scaledTempo = this->outs.scaledTempo;

    // about 400 msec
    _display();

    mux_process(&digital_mux);
    _processEncoders();

    this->eomBuffer += msDelta;
    if (this->eomBuffer > 5000000) this->eomBuffer = 5000000;
	this->latchPulseTimer += msDelta;
	if (this->latchPulseTimer > LATCH_PULSE_TIME) {
		this->latchPulseTimer = fmod(this->latchPulseTimer, LATCH_PULSE_TIME);
	}

	if (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) {
		this->modButtonFlashTimer += msDelta;
		if (this->modButtonFlashTimer > MOD_BUTTON_FLASH_TIME) {
			this->modButtonFlashCount++;
			this->modButtonFlashTimer = (this->modButtonFlashCount < MOD_BUTTON_FLASH_COUNT) ? 0.0 : MOD_BUTTON_FLASH_TIME;
		}
	}
};
