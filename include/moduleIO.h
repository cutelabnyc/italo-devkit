#include <Arduino.h>
#include "gpio.h"

#if MESSD_UP

#define NUM_INPUTS 5
#define NUM_OUTPUTS 4
#define NUM_ARGS (NUM_INPUTS + NUM_OUTPUTS)

// GPIO struct for hardware IO
pin_t GPIO_in[NUM_INPUTS] = {
	{A6, INPUT, ANALOG}, // Clock In
	{A3, INPUT, ANALOG}, // Downbeat in
	{A4, INPUT, ANALOG}, // Subdivision in
	{A7, INPUT, ANALOG}, // Phase in
	{7, INPUT, DIGITAL}   // Metric Modulation
};

pin_t GPIO_out[NUM_OUTPUTS] = {
	{4, OUTPUT, DIGITAL},  // Clock out,
	{12, OUTPUT, DIGITAL}, // Downbeat out
	{10, OUTPUT, DIGITAL}, // Subdivision out,
	{8, OUTPUT, DIGITAL}  // Phase out
};

#elif MISSED_OPPORTUNITIES
#endif
