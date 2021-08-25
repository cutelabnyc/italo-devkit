/**
 * main.cpp —— (Max Ardito, 07/09/20)
 *
 * Main functional loop for the Mess'd Up module. It's the
 * place where the hardware [GPIO_t] and the code API [/lib/] go on
 * expensive and indulgent, yet rewarding dinner dates, noshing on
 * the signals served by [buffer_t CV_in/CV_out].
 */

 // TODO: Define as a compiler flag
#define MESSD_UP 1

#include "gpio.h"

/**
 * NOTE: After refactoring all the code below,
 * try and get missed-opportunities in a similar place.
 * Then try and fix your cmake bugs in cutesynth, while
 * also moving the conditional macros to their own /modules
 * directory
 */

IO_buffer_t IO_buffer;

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup()
{
	GPIO_init(GPIO_in, NUM_INPUTS);
	GPIO_init(GPIO_out, NUM_OUTPUTS);

	Serial.begin(9600);

	MS_init(&messd);
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop()
{
	GPIO_read(GPIO_in, IO_buffer.in, NUM_INPUTS);

	// TODO: Define in CuteSynth
	MS_process(&messd, IO_buffer.in, IO_buffer.out);

	GPIO_write(GPIO_out, IO_buffer.out, NUM_OUTPUTS);
}
