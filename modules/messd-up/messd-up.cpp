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
            this->displayTempo = false;
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
            this->displayTempo = false;
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

    if (nextClockSwitch == LOW) {
        this->displayTempo = true;
        this->tempoDisplayTime = 0;
    } else {
        this->tempoDisplayTime += msDelta;
        if (this->tempoDisplayTime > 3000) this->displayTempo = false;
    }

    this->clockSwitch = nextClockSwitch;
}

void Module::_processModSwitch() {
    int nextModSwitch = digitalRead(this->modSwitchPin);
    this->modSwitch = nextModSwitch;
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

    for (int i = 0; i < 8; i++) {
        output_sr_val[i] = 0;
        leds_sr_val[i] = 0;
    }
}

void Module::process(float msDelta) {

    mux_process(&digital_mux);
    mux_process(&analog_mux);
    _processTapTempo(msDelta);
    _processEncoders();
    this->modSwitch = digitalRead(modSwitchPin);


    if (digitCounter == 4) {
        digitCounter = 0;
    }

    int value;

    switch (digitCounter) {
        case 0:
            if (this->displayTempo) {
                value = ((long) this->tapTempoOut) / 1000;
            } else {
                value = state.div / 10;
            }
            break;
        case 1:
            if (this->displayTempo) {
                value = (((long) this->tapTempoOut) / 100) % 10;
            } else {
                value = state.div % 10;
            }
            break;
        case 2:
            if (this->displayTempo) {
                value = (((long) this->tapTempoOut) / 10) % 10;
            } else {
                value = state.beats / 10;
            }
            break;
        case 3:
            if (this->displayTempo) {
                value = (long) this->tapTempoOut % 10;
            } else {
                value = state.beats % 10;
            }
            break;
    }
    seven_segment_process(&seven_segment_sr, digitCounter, value);
    digitCounter++;

    // Update the internal clock
    float phaseDelta = (this->tapTempoOut * msDelta) / (60000.0f);
    float clockPhase = phasor_step(&this->clock, phaseDelta);

    bool lastdiv = this->outs.subdivision;
    this->ins.delta = msDelta;
    this->ins.tempo = this->tapTempoOut;
    this->ins.beatsPerMeasure = this->state.beats;
    this->ins.subdivisionsPerMeasure = this->state.div;
    this->ins.phase = 0; // unused
    this->ins.ext_clock = clockPhase < 0.5;
    this->ins.modulationSignal = 0; // debug
    this->ins.modulationSwitch = this->modSwitch == LOW; // active low
    this->ins.latchChangesToDownbeat = false; // debug
    this->ins.latchModulationToDownbeat = true; // debug
    this->ins.invert = 0; // unused
    this->ins.isRoundTrip = analog_mux.outputs[AnalogMux.ROUND_SWITCH] < (MAX_VOLTAGE >> 1); // debug
    this->ins.reset = 0; // debug

    // compute wrap
    int wrapvoltage = this->analog_mux.outputs[AnalogMux.TRUNCATE_ATV];
    if (wrapvoltage >= MAX_VOLTAGE) {
        this->ins.truncation = 0;
    } else {
        float wrapfraction = ((float) wrapvoltage) / ((float) MAX_VOLTAGE);
        wrapfraction *= this->state.beats;
        wrapfraction = floorf(wrapfraction) + 1;
        this->ins.truncation = (int) wrapfraction;
    }
    this->ins.pulseWidth = 0.5; // debug

    MS_process(&this->messd, &this->ins, &this->outs);

    if (this->outs.eom) {
		this->eomBuffer = 0;
		this->animateModulateButtonTime = 0.0f;
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
    leds_sr_val[(uint8_t) LEDNames::ClockLEDButton] = this->clockSwitch; //debug
    leds_sr_val[(uint8_t) LEDNames::DownbeatLED] = this->outs.downbeat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLatchLED] = HIGH; //debug
    leds_sr_val[(uint8_t) LEDNames::DivLatchLED] = HIGH; //debug
    shift_register_process(&this->leds_sr, this->leds_sr_val, 8, true);

    /* this->outputBuffer[BEATS_OUT] = this->outs.beat; */
    /* this->outputBuffer[SUBDIVISIONS_OUT] = this->outs.subdivision; */
    /* this->outputBuffer[DOWNBEAT_OUT] = this->outs.downbeat; */
    /* this->outputBuffer[PHASE_OUT] = this->outs.phase; */

    this->eomBuffer += msDelta;
    if (this->eomBuffer > 5000000) this->eomBuffer = 5000000;
};
