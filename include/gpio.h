/**
 * gpio.h —— (Max Ardito, 07/10/20)
 *
 * Wrapper classs used in order to create a layer of abstraction
 * in between the hardware and the module's API. Consists of the general
 * structure for the pin layout used on the ATMEGA328, as well as
 * read/write wrapper functions based on the <Arduino.h> lib.
 */

#include <Arduino.h>
extern "C"
{
#include <cutesynth.h>
}
// Struct representing a single pin
typedef struct pin
{
	uint8_t pin;
	uint8_t mode;
	bool isAnalog;
} pin_t;

// Struct where all the IO data will be stored
typedef struct IO_buffer
{
	double in[NUM_INPUTS];
	double out[NUM_OUTPUTS];
} IO_buffer_t;

/**
 * Returns the global pin IO struct
 */
void GPIO_init(pin_t *self, uint8_t numArgs)
{
	for (int i = 0; i < numArgs; i++)
	{
		pinMode(self[i].pin, self[i].mode);
	}
}

/**
 * Reads incoming data from all inputs
 */
void GPIO_read(pin_t *self, double *inputValues, uint8_t numArgs)
{
	for (int i = 0; i < numArgs; i++)
	{
		if (self[i].isAnalog)
			inputValues[i] = analogRead(self[i].pin);
		else
			inputValues[i] = digitalRead(self[i].pin);
	}
}

/**
 * Writes data to all outputs
 */
void GPIO_write(pin_t *self, double *outputValues, uint8_t numArgs)
{
	for (int i = 0; i < numArgs; i++)
	{
		if (self[i].isAnalog)
			analogWrite(self[i].pin, outputValues[i]);
		else
			digitalWrite(self[i].pin, outputValues[i]);
	}
}
