/**
 * main.cpp —— (Max Ardito, 07/09/20)
 *
 * Main functional loop for the Mess'd Up module. It's the
 * place where the hardware [GPIO_t] and the code API [/lib/] go on
 * expensive and indulgent, yet rewarding dinner dates, noshing on
 * the signals served by [buffer_t CV_in/CV_out].
 */

#include "gpio.h"
#include "limits.h"

extern "C"
{
#include "messd.h"
}

messd_t messd;
GPIO_t GPIO;

uint16_t CLOCK_in;
uint16_t CLOCK_out;
uint16_t DOWNBEAT_in;
uint16_t DOWNBEAT_out;
uint16_t SUBDIVISION_in;
uint16_t SUBDIVISION_out;
uint16_t PHASE_in;
uint16_t PHASE_out;
bool METRIC_MODULATION_in;

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup()
{
	GPIO = GPIO_init();

	Serial.begin(9600);

	MS_init(&messd);
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop()
{
	GPIO_read(&GPIO, &CLOCK_in, &DOWNBEAT_in, &SUBDIVISION_in, &PHASE_in, &METRIC_MODULATION_in);

	MS_process(&messd,
		&CLOCK_in,
		&CLOCK_out,
		&DOWNBEAT_in,
		&DOWNBEAT_out,
		&SUBDIVISION_in,
		&SUBDIVISION_out,
		&PHASE_in,
		&PHASE_out,
		METRIC_MODULATION_in);

	GPIO_write(&GPIO, &CLOCK_out, &DOWNBEAT_out, &SUBDIVISION_out, &PHASE_out);
}
