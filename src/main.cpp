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

// consts
int beatsDivMin = 2;
int beatsDivMax = 32;

// Mux controllers
int m0 = 8;
int m1 = 9;
int m2 = 10;

// Analog inputs
int aIn0 = A1;
int aIn1 = A2;
int aVal0 = 0;
int aVal1 = 0;

// Storage for the encoders
int divEncA = 0;
int divEncB = 0;
int encStateDiv = 0;
int beatsEncA = 0;
int beatsEncB = 0;
int encStateBeats = 0;

// State
struct state {
	int beats = 4;
	int div = 7;
} state;

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

	pinMode(m0, OUTPUT);
	pinMode(m1, OUTPUT);
	pinMode(m2, OUTPUT);

	module->init();

	lastMillis = micros();
}

int makeEncoderState(int pinA, int pinB) {
	return (pinB << 1) | pinA;
}

int FeedState(int &oldState, int newState)
{
	int out = 0;

	if (oldState != newState) {
		int isFwd = ((newState == 0) && (oldState == 1)) ||
			((newState == 1) && (oldState == 3)) ||
			((newState == 3) && (oldState == 2)) ||
			((newState == 2) && (oldState == 0));
		out = isFwd ? 1 : -1;
	}

	oldState = newState;
	return out;
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
    
    // GPIO_write(module->getOutputPinSchematic(), module->getOutputBuffer(),
    //            module->getNumOutputs());

	// Hardcoded stuff outside of GPIO_write

	// Read the encoder
	for (int i = 0; i < 8; i++) {
		digitalWrite(m0, i & 1);
		digitalWrite(m1, (i >> 1) & 1);
		digitalWrite(m2, (i >> 2) & 1);

		aVal1 = analogRead(aIn1);

		// Serial.print(i);
		// Serial.print(": ");
		// Serial.print(aVal0);
		// Serial.print("\t");

		if (i == 4) {
			divEncA = aVal1 < 512;
		} else if (i == 5) {
			divEncB = aVal1 < 512;
		} else if (i == 6) {
			beatsEncA = aVal1 < 512;
		} else if (i == 7) {
			beatsEncB = aVal1 < 512;
		}
	}

	int newState = makeEncoderState(beatsEncA, beatsEncB);
	int inc = FeedState(encStateBeats, newState);
	if (inc != 0) {
		state.beats -= inc;
		if (state.beats < beatsDivMin) state.beats = beatsDivMax;
		if (state.beats > beatsDivMax) state.beats = beatsDivMin;
	}
	newState = makeEncoderState(divEncA, divEncB);
	inc = FeedState(encStateDiv, newState);
	if (inc != 0) {
		state.div -= inc;
		if (state.div < beatsDivMin) state.div = beatsDivMax;
		if (state.div > beatsDivMax) state.div = beatsDivMin;
	}

	// Update module state
	double *ins = module->getInputBuffer();
	ins[BEATS] = state.beats;
	ins[SUBDIVISIONS] = state.div;

	// Process the module
	module->process(delta);

	// Write the display
	char s[2];
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

			// Beat ones
			if (currentDig == 3) {
				digitalWrite(dataPinA, digitDisplay[state.beats % 10][i]);
			} else if (currentDig == 2) {
				digitalWrite(dataPinA, digitDisplay[state.beats / 10][i]);
			} else if (currentDig == 1) {
				digitalWrite(dataPinA, digitDisplay[state.div % 10][i]);
			} else {
				digitalWrite(dataPinA, digitDisplay[state.div / 10][i]);
			}
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
