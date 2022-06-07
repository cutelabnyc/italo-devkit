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

Module::Module() { MS_init(&this->messd); };

void Module::init() {
    // Analog read pin for the digital mux
    pinMode(digitalMuxIn, INPUT);

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

    this->ins.delta = msDelta;
    this->ins.tempo = 120; // debug
    this->ins.beatsPerMeasure = this->state.beats;
    this->ins.subdivisionsPerMeasure = this->state.div;
    this->ins.phase = 0; // debug

    this->ins.ext_clock = 0; // debug
    this->ins.truncation = 0; // debug
    this->ins.metricModulation = 0; // debug
    this->ins.latchChangesToDownbeat = 0; // debug
    this->ins.invert = 0; // debug
    this->ins.isRoundTrip = 0; // debug
    this->ins.reset = 0; // debug

    this->ins.wrap = 0; // debug
    this->ins.pulseWidth = 0.5; // debug

    /* _scaleValues(); */

    MS_process(&this->messd, &this->ins, &this->outs);

    // Configure outputs
    output_sr_val[(uint8_t) OutputNames::Nothing] = HIGH;
    output_sr_val[(uint8_t) OutputNames::TruncateLED] = HIGH; // debug
    output_sr_val[(uint8_t) OutputNames::DivLED] = HIGH; // debug this->outs.subdivision ? LOW : HIGH;
    output_sr_val[(uint8_t) OutputNames::EoMOutput] = HIGH; // debug
    output_sr_val[(uint8_t) OutputNames::TruncateOutput] = HIGH; // debug
    output_sr_val[(uint8_t) OutputNames::DivOutput] = HIGH; // debug this->outs.subdivision ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::DownbeatOutput] = HIGH; // debug this->outs.downbeat ? HIGH : LOW;
    output_sr_val[(uint8_t) OutputNames::BeatOutput] = HIGH; // debug this->outs.beat ? HIGH : LOW;
    shift_register_process(&output_sr, this->output_sr_val, 8, true);

    // for (int i = 0; i < 8; i++)
    // 	this->output_sr_val[i] = LOW;
    // shift_register_process(&this->output_sr, this->output_sr_val, 8);

    // Configure LEDs
    leds_sr_val[(uint8_t) LEDNames::Nothing] = HIGH;
    leds_sr_val[(uint8_t) LEDNames::ModLEDButton] = HIGH; // debug this->outs.modulate ? HIGH : LOW;
    leds_sr_val[(uint8_t) LEDNames::EoMLED] = HIGH; // debug
    leds_sr_val[(uint8_t) LEDNames::ClockLEDButton] = this->clockSwitch; //debug
    leds_sr_val[(uint8_t) LEDNames::DownbeatLED] = HIGH; // debug this->outs.downbeat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLED] = HIGH; // debug this->outs.beat ? LOW : HIGH;
    leds_sr_val[(uint8_t) LEDNames::BeatLatchLED] = LOW; //debug
    leds_sr_val[(uint8_t) LEDNames::DivLatchLED] = HIGH; //debug

    // for (int i = 0; i < 8; i++)
    // 	this->leds_sr_val[i] = HIGH;
    shift_register_process(&this->leds_sr, this->leds_sr_val, 8, true);

    // digitalWrite(latchPinLEDs, LOW);

    // for (int i = 0; i < 8; i++) {
    // 	digitalWrite(clockPinLEDs, LOW);
    // 	digitalWrite(dataPinLEDs, this->leds_sr_val[i]);
    // 	digitalWrite(clockPinLEDs, HIGH);
    // }

    // digitalWrite(latchPinLEDs, HIGH);

    /* this->outputBuffer[BEATS_OUT] = this->outs.beat; */
    /* this->outputBuffer[SUBDIVISIONS_OUT] = this->outs.subdivision; */
    /* this->outputBuffer[DOWNBEAT_OUT] = this->outs.downbeat; */
    /* this->outputBuffer[PHASE_OUT] = this->outs.phase; */
};
