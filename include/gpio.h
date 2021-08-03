/**
 * gpio.h —— (Max Ardito, 07/10/20)
 *
 * Wrapper classs used in order to create a layer of abstraction
 * in between the hardware and the module's API. Consists of the general
 * structure for the pin layout used on the ATMEGA328, as well as
 * read/write wrapper functions based on the <Arduino.h> lib.
 */

#include "globals.h"
#include <Arduino.h>

typedef uint8_t pin_t;

/**
 * Struct representing the entire IO for the module
 */
typedef struct GPIO
{
	pin_t CLOCK_KNOB;
	pin_t CLOCK_OUT;
	pin_t DOWNBEAT_IN;
	pin_t DOWNBEAT_OUT;
	pin_t SUBDIVISION_IN;
	pin_t SUBDIVISION_OUT;
	pin_t PHASE_IN;
	pin_t PHASE_OUT;
	pin_t METRIC_MODULATION;
} GPIO_t;

/**
 * Returns the global pin IO struct
 */
GPIO_t GPIO_init(void)
{
	// GPIO_t self = {
	// 	{A5, A0, A1, A2}, // CV Ins -- AD5, AD0, AD1, AD2
	// 	{4, 12, 10, 8},   // CV Outs -- PD4, PB4, PB2, PB0
	// 	A3,               // Reset In -- AD3
	// 	A7,               // Density In -- AD4
	// 	3,                // Pulse out -- PD3
	// 	{6, 5},           // Reset and Density LEDs (respectively) -- PD6, PD5,
	// 	7,				  // PD7 used to set miss/match
	// 	{13, 11, 9}       // "Missed" Opportunities —- PB5, PB3, PB1
	// };

	GPIO_t self = {
		A6, // Clock In
		4,  // Clock out,
		A3, // Downbeat in
		12, // Downbeat out
		A4, // Subdivision in
		10, // Subdivision out,
		A7, // Phase in
		8,  // Phase out
		7   // Metric Modulation

	};

	pinMode(self.CLOCK_KNOB, INPUT);
	pinMode(self.CLOCK_OUT, OUTPUT);
	pinMode(self.DOWNBEAT_IN, INPUT);
	pinMode(self.DOWNBEAT_OUT, OUTPUT);
	pinMode(self.SUBDIVISION_IN, INPUT);
	pinMode(self.SUBDIVISION_OUT, OUTPUT);
	pinMode(self.PHASE_IN, INPUT);
	pinMode(self.PHASE_OUT, OUTPUT);
	pinMode(self.METRIC_MODULATION, INPUT);

	return self;
}

/**
 * Reads incoming data from all inputs
 */
void GPIO_read(GPIO_t *self,
	uint16_t *clock,
	uint16_t *downbeat_in,
	uint16_t *subdivision_in,
	uint16_t *phase_in,
	bool *metric_modulation)
{
	*clock = analogRead(self->CLOCK_KNOB);
	*downbeat_in = analogRead(self->DOWNBEAT_IN);
	*subdivision_in = analogRead(self->SUBDIVISION_IN);
	*phase_in = analogRead(self->PHASE_IN);
	*metric_modulation = digitalRead(self->METRIC_MODULATION);
}

/**
 * Writes data to all outputs
 */
void GPIO_write(GPIO_t *self,
	uint16_t *clock_out,
	uint16_t *downbeat_out,
	uint16_t *subdivision_out,
	uint16_t *phase_out)
{
	digitalWrite(self->CLOCK_OUT, *clock_out);
	digitalWrite(self->DOWNBEAT_OUT, *downbeat_out);
	digitalWrite(self->SUBDIVISION_OUT, *subdivision_out);
	digitalWrite(self->PHASE_OUT, *phase_out);
}
