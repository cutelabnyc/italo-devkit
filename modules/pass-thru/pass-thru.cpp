#include "pass-thru.hpp"

static uint16_t magic = 42;

Module::Module() {

}

void Module::HardwareRead(passthru_ins_t *ins, passthru_outs_t *outs) {}
void Module::HardwareWrite(passthru_ins_t *ins, passthru_outs_t *outs) {}

void Module::initHardware() {

}

void Module::process(float msDelta) {

}
