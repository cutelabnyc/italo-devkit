/**
 * main.cpp —— (Max Ardito, 07/09/20)
 *
 * Main functional loop for the Mess'd Up module. It's the
 * place where the hardware [GPIO_t] and the code API [/lib/] go on
 * expensive and indulgent, yet rewarding dinner dates, noshing on
 * the signals served by [buffer_t CV_in/CV_out].
 */

// #include <interfaces.hpp>
#include <../modules/messd-up/messd-up.cpp>

ModuleInterface *module = buildModule();
unsigned long previousTime;

// This could eventually move to GPIO_init I suppose

// Seven segment
int dataPinA = 7;
int latchPinA = 6;
int clockPinA = 5;
int dataPinB = 13;
int latchPinB = 12;
int clockPinB = 11;
int millisAccum = 0;
int lastMillis = 0;
int millisUpdateInterval = 2000;
int currentDig = 0;
int maxDig = 4;

const int digitDisplay[10][8] {
	// NUMBERS
	{0,1,1,1,1,1,1,0}, //ZERO
	{0,0,0,0,1,1,0,0}, //ONE
	{1,0,1,1,0,1,1,0}, //TWO
	{1,0,0,1,1,1,1,0}, //THREE
	{1,1,0,0,1,1,0,0}, //FOUR
	{1,1,0,1,1,0,1,0}, //FIVE
	{1,1,1,1,1,0,1,0}, //SIX
	{0,0,0,0,1,1,1,0}, //SEVEN
	{1,1,1,1,1,1,1,0}, //EIGHT
	{1,1,0,1,1,1,1,0}, //NINE
};

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup()
{
    GPIO_init(module->getInputPinSchematic(), module->getNumInputs());
    GPIO_init(module->getOutputPinSchematic(), module->getNumOutputs());
    
    Serial.begin(9600);
    previousTime = micros();

	pinMode(clockPinA, OUTPUT);
	pinMode(latchPinA, OUTPUT);
	pinMode(dataPinA, OUTPUT);

	pinMode(clockPinB, OUTPUT);
	pinMode(latchPinB, OUTPUT);
	pinMode(dataPinB, OUTPUT);

	module->init();

	lastMillis = micros();
}

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop()
{
    int startMillis = micros();
    int delta = startMillis - previousTime;
    previousTime = startMillis;

	// Serial.println(delta);
    
    // GPIO_read(module->getInputPinSchematic(), module->getInputBuffer(),
    //           module->getNumInputs());
    
    module->process(delta);
    
    // GPIO_write(module->getOutputPinSchematic(), module->getOutputBuffer(),
    //            module->getNumOutputs());

	// Hardcoded stuff outside of GPIO_write
	int thisMillis = micros();
	millisAccum += thisMillis - lastMillis;

	if (millisAccum > millisUpdateInterval) {
		millisAccum = millisAccum % millisUpdateInterval;

		// Disable all digits
		// digitalWrite(latchPinA, LOW);
		// for (int i = 0; i < 16; i++) {
		// 	digitalWrite(clockPinA, LOW);
		// 	digitalWrite(dataPinA, LOW);
		// 	digitalWrite(clockPinA, HIGH);
		// }
		// digitalWrite(latchPinA, HIGH);

		// Enable the digit of interest
		digitalWrite(latchPinA, LOW);
		for (int i = 0; i < 8; i++) {
			digitalWrite(clockPinA, LOW);
			digitalWrite(dataPinA, i == currentDig ? HIGH : LOW);
			digitalWrite(clockPinA, HIGH);
		}

		// Write the digit
		for (int i = 0; i < 8; i++) {
			digitalWrite(clockPinA, LOW);
			// digitalWrite(dataPinA, _numMatrix[0][i]);
			digitalWrite(dataPinA, digitDisplay[currentDig][i]);
			digitalWrite(clockPinA, HIGH);
		}

		digitalWrite(latchPinA, HIGH);

		currentDig = (currentDig + 1) % 4;
	}

	// Write the output pins
	double *ob = module->getOutputBuffer();
	int outvals[4] = { 0, 4, 6, 7 };
	digitalWrite(latchPinB, LOW);
	for (int i = 0; i < 16; i++) {
		int revPin = 15 - i;
		digitalWrite(clockPinB, LOW);

		if (revPin == 0) {
			digitalWrite(dataPinB, ob[0] < 0.5 ? HIGH : LOW);
		} else if (revPin == 4) {
			digitalWrite(dataPinB, ob[1] < 0.5 ? HIGH : LOW);
		} else if (revPin == 6) {
			digitalWrite(dataPinB, ob[2] < 0.5 ? HIGH : LOW);
		} else if (revPin == 7) {
			digitalWrite(dataPinB, ob[3] < 0.5 ? HIGH : LOW);
		} else {
			digitalWrite(dataPinB, HIGH);
		}

		digitalWrite(clockPinB, HIGH);
	}
	digitalWrite(latchPinB, HIGH);

	lastMillis = thisMillis;
}
