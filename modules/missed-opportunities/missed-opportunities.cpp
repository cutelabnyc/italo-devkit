#include "missed-opportunities.hpp"

void Module::_scaleValues() {}

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

Module::Module(){};

void Module::init() {
  // Analog read pin for the digital mux
  pinMode(digitalMuxIn, INPUT);

  // Setting all of the shift register pins to be outputs
  pinMode(clockPinA, OUTPUT);
  pinMode(latchPinA, OUTPUT);
  pinMode(dataPinA, OUTPUT);

  // Mux selector pins are outputs
  pinMode(m0, OUTPUT);
  pinMode(m1, OUTPUT);
  pinMode(m2, OUTPUT);
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
  seven_segment_process(&shift_register, digitCounter, value);
  digitCounter++;
  /* this->ins.delta = msDelta / 1000.0; */
  /* this->ins.tempo = this->inputBuffer[TEMPO]; */
  /* this->ins.tempo = this->inputBuffer[DEBUG_TEMPO]; */
  /* this->ins.beatsPerMeasure = this->inputBuffer[BEATS]; */
  /* this->ins.subdivisionsPerMeasure = this->inputBuffer[SUBDIVISIONS]; */
  // this->ins.phase = this->inputBuffer[PHASE];
  // this->ins.beatsPerMeasure = 4;
  // this->ins.subdivisionsPerMeasure = 7;
  /* this->ins.phase = 0; */

  /* this->ins.ext_clock = 0; */
  /* this->ins.truncation = 0; */
  /* this->ins.metricModulation = 0; */
  /* this->ins.latchChangesToDownbeat = 0; */
  /* this->ins.invert = 0; */
  /* this->ins.isRoundTrip = 0; */
  /* this->ins.reset = 0; */

  /* this->ins.wrap = 0; */
  /* this->ins.pulseWidth = 0.5; */

  /* _scaleValues(); */

  /* MS_process(&this->messd, &this->ins, &this->outs); */

  /* this->outputBuffer[BEATS_OUT] = this->outs.beat; */
  /* this->outputBuffer[SUBDIVISIONS_OUT] = this->outs.subdivision; */
  /* this->outputBuffer[DOWNBEAT_OUT] = this->outs.downbeat; */
  /* this->outputBuffer[PHASE_OUT] = this->outs.phase; */
};
