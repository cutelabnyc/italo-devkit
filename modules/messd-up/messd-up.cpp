#include "messd-up.hpp"

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
            this->tapTempoOut += inc;
            if (this->tapTempo < Module::tempoMin) this->tapTempo = this->tapTempoOut = Module::tempoMin;
            if (this->tapTempo > Module::tempoMax) this->tapTempo = this->tapTempoOut = Module::tempoMax;
        } else {
            this->tempoDisplayTime = TEMPO_DISPLAY_TIME;
            state.beats += inc;
            if (state.beats < beatsDivMin)
                state.beats = beatsDivMax;
            if (state.beats > beatsDivMax)
                state.beats = beatsDivMin;
        }
    }
    inc = encoder_process(&div_encoder,
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_A],
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_B]);
    if (inc != 0) {
        // When you hold down the clock button, you set the tempo with the knobs
        if (this->clockSwitch == LOW) {
            this->tapTempo += inc * 10;
            this->tapTempoOut += inc * 10;
            if (this->tapTempo < Module::tempoMin) this->tapTempo = this->tapTempoOut = Module::tempoMin;
            if (this->tapTempo > Module::tempoMax) this->tapTempo = this->tapTempoOut = Module::tempoMax;
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
            this->tapTempoOut = this->tapTempo;
            totalTaps = totalTaps >= 5 ? 5 : (totalTaps + 1);
        }

        this->lastTapMicros = nextTapMicros;

        if (this->tapTempo < Module::tapTempoMin) this->tapTempo = this->tapTempoOut = Module::tapTempoMin;
        if (this->tapTempo > Module::tapTempoMax) this->tapTempo = this->tapTempoOut = Module::tapTempoMax;
    }

    if (nextClockSwitch != LOW) {
        this->tempoDisplayTime += msDelta;
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

void Module::_display() {
	if (digitCounter == 4) {
        digitCounter = 0;
    }

    int value;
	long displayableTempo = (long) floor(this->scaledTempo);

	if (this->outs.resetPending) {
		this->displayState = DisplayState::Pop;
	} else if (this->clockSwitch == LOW || this->tempoDisplayTime < TEMPO_DISPLAY_TIME) {
		this->displayState = DisplayState::Tempo;
	} else {
		this->displayState = DisplayState::Default;
	}

    switch (digitCounter) {
        case 0:
            if (this->displayState == DisplayState::Tempo) {
                value = (displayableTempo) / 1000;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeDiv / 10;
            } else if (this->displayState == DisplayState::Pop) {
				value = (int) SpecialDigits::Nothing;
			}
            break;
        case 1:
            if (this->displayState == DisplayState::Tempo) {
                value = ((displayableTempo) / 100) % 10;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeDiv % 10;
            } else if (this->displayState == DisplayState::Pop) {
				value = (int) SpecialDigits::P;
			}
            break;
        case 2:
            if (this->displayState == DisplayState::Tempo) {
                value = ((displayableTempo) / 10) % 10;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeBeats / 10;
            } else if (this->displayState == DisplayState::Pop) {
				value = 0;
			}
            break;
        case 3:
            if (this->displayState == DisplayState::Tempo) {
                value = displayableTempo % 10;
            } else if (this->displayState == DisplayState::Default) {
                value = state.activeBeats % 10;
            } else if (this->displayState == DisplayState::Pop) {
				value = (int) SpecialDigits::P;
			}
            break;
    }
    seven_segment_process(&seven_segment_sr, digitCounter, value);
    digitCounter++;
}

Module::Module() {
    MS_init(&this->messd);
    phasor_init(&this->clock);
};

void Module::init() {

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
}

void Module::process(float msDelta) {

	// Update the internal clock
    float phaseDelta = (this->tapTempoOut * msDelta) / (60000.0f);
    float clockPhase = phasor_step(&this->clock, phaseDelta);
	digitalWrite(clockOut, clockPhase < 0.5 ? HIGH : LOW);

#ifdef FORCE_INTERNAL_CLOCK
	uint8_t clockInput = clockPhase < 0.5;
#else
	uint8_t clockInput = digitalRead(clockIn);
#endif

    mux_process(&digital_mux);
    mux_process(&analog_mux);
    _processTapTempo(msDelta);
    _processEncoders();
	_processModSwitch(msDelta);

    // Handle divide and beat switches
    if (digital_mux.outputs[DigitalMux.DIV_SWITCH] == LOW) {
        if (div_switch_state_prev == HIGH) div_latch = !div_latch;
    }
    div_switch_state_prev = digital_mux.outputs[DigitalMux.DIV_SWITCH];
    if (digital_mux.outputs[DigitalMux.BEAT_SWITCH] == LOW) {
        if (beat_switch_state_prev == HIGH) beat_latch = !beat_latch;
    }
    beat_switch_state_prev = digital_mux.outputs[DigitalMux.BEAT_SWITCH];

    // Compute the final value for subdivisions and beats based on modulation inputs
    // and attenuverters
    float divOffset = 1.0f - (float) this->analog_mux.outputs[AnalogMux.DIVIDE_INPUT] / (float) MAX_VOLTAGE;
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

    this->ins.delta = msDelta;
    this->ins.tempo = this->tapTempoOut;
    this->ins.beatsPerMeasure = state.activeBeats;
    this->ins.subdivisionsPerMeasure = state.activeDiv;
    this->ins.phase = 0; // unused
    this->ins.ext_clock = clockInput == HIGH;
    this->ins.modulationSignal = 0; // debug
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
#ifndef IS_POWERED_FROM_ARDUINO
	truncationOffset -= 0.5f;
	truncationOffset *= 2.0f;
#endif
    baseTruncation = fmax(0.0, fmin(1.0, baseTruncation + truncationOffset));
	this->ins.truncation = baseTruncation >= 1.0f ? -1.0f : baseTruncation;
    this->ins.pulseWidth = 0.5; // debug

    MS_process(&this->messd, &this->ins, &this->outs);

    if (this->outs.eom) {
        this->eomBuffer = 0;
        this->animateModulateButtonTime = 0.0f;
        this->tempoDisplayTime = 0;
		this->state.div = this->ins.subdivisionsPerMeasure;
    }

    // Animate the modulation button
    bool modButtonOn = false;
    if (this->outs.modulationPending) {
        modButtonOn = this->animateModulateButtonTime < MOD_BUTTON_STROBE_SLOW;
        this->animateModulateButtonTime += msDelta;
        if (this->animateModulateButtonTime > (2.0f * MOD_BUTTON_STROBE_SLOW)) {
            this->animateModulateButtonTime = fmodf(this->animateModulateButtonTime, (2.0f * MOD_BUTTON_STROBE_SLOW));
        }
    } else if (this->outs.inRoundTripModulation) {
        modButtonOn = true;
    }

    // Configure outputs
    output_sr_val[(uint8_t) OutputNames::Nothing] = HIGH;
    output_sr_val[(uint8_t) OutputNames::TruncateLED] = this->outs.truncate ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::DivLED] = this->outs.subdivision ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::EoMOutput] = this->eomBuffer < EOM_BUFFER_MS ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::TruncateOutput] = this->outs.truncate ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::DivOutput] = this->outs.subdivision ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::DownbeatOutput] = this->outs.downbeat ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::BeatOutput] = this->outs.beat ? HIGH : LOW;
    shift_register_process(&output_sr, this->output_sr_val, 8, true);

    // Configure LEDs
    leds_sr_val[(uint8_t) LEDNames::Nothing] = HIGH;
    leds_sr_val[(uint8_t) LEDNames::ModLEDButton] = modButtonOn ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::EoMLED] = this->eomBuffer < EOM_LED_BUFFER_MS ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::ClockLEDButton] = clockPhase < 0.5 ? LOW : HIGH; // clockInput == HIGH ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::DownbeatLED] = this->outs.downbeat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLatchLED] = this->beat_latch ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::DivLatchLED] = this->div_latch ? LOW : HIGH;
    shift_register_process(&this->leds_sr, this->leds_sr_val, 8, true);

	this->scaledTempo = this->outs.scaledTempo;

	_display();

    this->eomBuffer += msDelta;
    if (this->eomBuffer > 5000000) this->eomBuffer = 5000000;
};
