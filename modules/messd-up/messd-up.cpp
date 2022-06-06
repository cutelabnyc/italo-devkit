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
        state.beats += inc;
        if (state.beats < beatsDivMin)
            state.beats = beatsDivMax;
        if (state.beats > beatsDivMax)
            state.beats = beatsDivMin;
    }
    inc = encoder_process(&div_encoder,
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_A],
        digital_mux.outputs[DigitalMux.DIVIDE_ENC_B]);
    if (inc != 0) {
        state.div += inc;
        if (state.div < beatsDivMin)
            state.div = beatsDivMax;
        if (state.div > beatsDivMax)
            state.div = beatsDivMin;
    }
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

    _processEncoders();
    mux_process(&digital_mux);
    mux_process(&analog_mux);

    if (digitCounter == 4) {
		digitCounter = 0;
    }

    int value;

    switch (digitCounter) {
		case 0:
			value = state.div / 10;
			break;
		case 1:
			value = state.div % 10;
			break;
		case 2:
			value = state.beats / 10;
			break;
		case 3:
			value = state.beats % 10;
			break;
    }
    seven_segment_process(&seven_segment_sr, digitCounter, value);
    digitCounter++;

    this->ins.delta = msDelta / 1000.0;
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
	output_sr_val[(uint8_t) OutputNames::Nothing] = LOW;
	output_sr_val[(uint8_t) OutputNames::TruncateLED] = HIGH; // debug
	output_sr_val[(uint8_t) OutputNames::DivLED] = this->outs.subdivision ? LOW : HIGH;
	output_sr_val[(uint8_t) OutputNames::EoMOutput] = LOW; // debug
	output_sr_val[(uint8_t) OutputNames::TruncateOutput] = LOW; // debug
	output_sr_val[(uint8_t) OutputNames::DivOutput] = this->outs.subdivision ? HIGH : LOW;
	output_sr_val[(uint8_t) OutputNames::DownbeatOutput] = this->outs.downbeat ? HIGH : LOW;
	output_sr_val[(uint8_t) OutputNames::BeatOutput] = this->outs.beat ? HIGH : LOW;
	shift_register_process(&output_sr, this->output_sr_val, 8);

	// for (int i = 0; i < 8; i++)
	// 	this->output_sr_val[i] = LOW;
	// shift_register_process(&this->output_sr, this->output_sr_val, 8);

	// Configure LEDs
	leds_sr_val[(uint8_t) LEDNames::Nothing] = LOW;
	leds_sr_val[(uint8_t) LEDNames::ModLEDButton] = this->outs.modulate ? LOW : HIGH;
	leds_sr_val[(uint8_t) LEDNames::EoMLED] = HIGH; // debug
	leds_sr_val[(uint8_t) LEDNames::ClockLEDButton] = HIGH; //debug
	leds_sr_val[(uint8_t) LEDNames::DownbeatLED] = this->outs.downbeat ? LOW : HIGH;
	leds_sr_val[(uint8_t) LEDNames::BeatLED] = this->outs.beat ? LOW : HIGH;
	leds_sr_val[(uint8_t) LEDNames::BeatLatchLED] = HIGH; //debug
	leds_sr_val[(uint8_t) LEDNames::DivLatchLED] = HIGH; //debug

	// for (int i = 0; i < 8; i++)
	// 	this->leds_sr_val[i] = HIGH;
	shift_register_process(&this->leds_sr, this->leds_sr_val, 8);

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
