#pragma once

#ifndef INTERFACE_H
#define INTERFACE_H

#include <gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

	// using namespace std;

	// template <typename M, typename Ins, typename Outs>
	// class ModuleInterface {
	// private:
	// 	uint8_t numInputs;
	// 	uint8_t numOutputs;

	// public:
	// 	virtual void init(M self) = 0;
	// 	virtual void process(M self, Ins ins, Outs outs) = 0;

	// 	pin_t *GPIO_in;
	// 	pin_t *GPIO_out;
	// };

	uint8_t numInputs;
	uint8_t numOutputs;

	pin_t *GPIO_in;
	pin_t *GPIO_out;

	void main_init();
	void main_process();

#ifdef __cplusplus
}
#endif

#endif // INTERFACE_H
