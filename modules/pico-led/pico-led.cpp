#include "pico-led.hpp"

#ifdef __cplusplus
extern "C" {
#endif

#include "./examples/LCD_Test.h"

#ifdef __cplusplus
}
#endif

static uint16_t magic = 42;

Module::Module() {

}

void Module::HardwareRead(picoled_ins_t *ins, picoled_outs_t *outs) {}
void Module::HardwareWrite(picoled_ins_t *ins, picoled_outs_t *outs) {}

void Module::initHardware() {

}

void Module::process(float msDelta) {
	LCD_0in96_test();
}
