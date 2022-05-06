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

// Shift register pins
// old arduino values
// int dataPinA = 7;
// int latchPinA = 6;
// int clockPinA = 5;

// int dataPinB = 13;
// int latchPinB = 12;
// int clockPinB = 11;

// Shift register pins (seven segment)
int dataPinA = 11; //PB3
int latchPinA = 9; //PB1
int clockPinA = 10; //PB2
int latchPinA2 = 13; //PB5
int clockPinA2 = 12; //PB4

// The second shift register (outs + LEDs)
int dataPinB = 7; //PD7
int latchPinB = 6; //PD6
int clockPinB = 5; //PD5
struct ShiftOutputsB {
	int NOTHING = 0;
	int TRUNCATE_LED = 1;
	int DIVIDE_LED = 2;
	int EoM_OUTPUT = 3;
	int TRUNCATE_OUTPUT = 4;
	int DIVIDE_OUTPUT = 5;
	int DOWNBEAT_OUTPUT = 6;
	int BEAT_OUTPUT = 7;
} ShiftOutputsB;

// The third shift register (LEDs)
int dataPinC = 8; //PB0
int latchPinC = A4; //PC4
int clockPinC = A5; //PC5
struct ShiftOutputsC {
	int MOD_LED = 0;
	int EoM_LED = 1;
	int CLOCK_LED = 2;
	int DOWNBEAT_LED = 3;
	int BEAT_LED = 4;
	int BEAT_LATCH_LED = 5;
	int DIV_LATCH_LED = 6;
	int NOTHING = 7;
} ShiftOutputsC;

int millisAccum = 0;
int lastMillis = 0;
int millisUpdateInterval = 2;
int currentDig = 0;
int maxDig = 4;

// consts
int beatsDivMin = 2;
int beatsDivMax = 32;

// Mux controller pins
int m0 = 4; //PD4
int m1 = 3; //PD3
int m2 = 2; //PD2
int analogMuxIn = A0; //PC0
int digitalMuxIn = A2; //PC2

struct AnalogMux {
	int DIVIDE_ATV = 0;
	int TRUNCATE_ATV = 1;
	int LATCH_SWITCH = 2;
	int BEAT_INPUT = 3;
	int ROUND_SWITCH = 4;
	int DIVIDE_INPUT = 5;
	int MOD_INPUT = 6;
	int TRUNCATE_INPUT = 7;
} AnalogMux;
struct DigitalMux {
	int BEAT_ENC_B = 0;
	int BEAT_SWITCH = 1;
	int DIV_SWITCH = 2;
	int BEAT_ENC_A = 3;
	int DIVIDE_ENC_B = 4;
	int CLOCK_IN = 5;
	int DIVIDE_ENC_A = 6;
	int CLOCK_SWITCH = 7;
} DigitalMux;

// Storage for analog/digital
int analogVal = 0;
int digitalVal = 0;

// Storage for the encoders
int divEncA = 0;
int divEncB = 0;
int encStateDiv = 0;
int beatsEncA = 0;
int beatsEncB = 0;
int encStateBeats = 0;

// Storage for the pots
int pot0 = 0;
int pot1 = 0;

// Storage for the latch switches
int beat_latch = 0;
int div_latch = 0;
int beat_switch_state_prev = 0;
int div_switch_state_prev = 0;

// State
struct state {
	int beats = 4;
	int div = 7;
} state;

// Shift register outputs
// q0: truncate LED
// q1: truncate out
// q2: divide out
// q3: downbeat out
// q4: downbeat LED
// q5: beat out
// q6: beat LED
// q7: divide LED

const int digitDisplay[10][8] {
	// NUMBERS
	{0,0,1,1,1,1,1,1}, //ZERO
	{0,0,0,0,0,1,1,0}, //ONE
	{0,1,0,1,1,0,1,1}, //TWO
	{0,1,0,0,1,1,1,1}, //THREE
	{0,1,1,0,0,1,1,0}, //FOUR
	{0,1,1,0,1,1,0,1}, //FIVE
	{0,1,1,1,1,1,0,1}, //SIX
	{0,0,0,0,0,1,1,1}, //SEVEN
	{0,1,1,1,1,1,1,1}, //EIGHT
	{0,1,1,0,1,1,1,1}, //NINE
};

/**
 * Initializes the ATMEGA328's pins, initializes
 * the other structs' variables, starts off the Serial
 * monitor for possible debugging
 **/
void setup()
{
    // GPIO_init(module->getInputPinSchematic(), module->getNumInputs());
    // GPIO_init(module->getOutputPinSchematic(), module->getNumOutputs());
    
    Serial.begin(9600);
    previousTime = micros();

	Serial.println("hi I'm a serial");

	pinMode(digitalMuxIn, INPUT);

	pinMode(clockPinA, OUTPUT);
	pinMode(latchPinA, OUTPUT);
	pinMode(clockPinA2, OUTPUT);
	pinMode(latchPinA2, OUTPUT);
	pinMode(dataPinA, OUTPUT);

	pinMode(clockPinB, OUTPUT);
	pinMode(latchPinB, OUTPUT);
	pinMode(dataPinB, OUTPUT);

	pinMode(clockPinC, OUTPUT);
	pinMode(latchPinC, OUTPUT);
	pinMode(dataPinC, OUTPUT);

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

static bool ff = false;


/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop()
{
    int startMillis = micros();
    int delta = (startMillis - previousTime) / 1000;
    previousTime = startMillis;

	// Write the display
	int thisMillis = micros();
	millisAccum += thisMillis - lastMillis;

	if (!ff) {
		millisAccum = millisAccum % millisUpdateInterval;

		Serial.println("here");
		currentDi

		digitalWrite(latchPinA, LOW);
		digitalWrite(latchPinA2, LOW);

		// Enable the digit of interest
		for (int i = 5; i >= 0; i--) {
			digitalWrite(clockPinA, LOW);
			digitalWrite(clockPinA2, LOW);
			digitalWrite(dataPinA, i == currentDig ? HIGH : LOW);
			// digitalWrite(dataPinA, HIGH);
			digitalWrite(clockPinA, HIGH);
			digitalWrite(clockPinA2, HIGH);
		}

		// Write the digit
		for (int i = 0; i < 8; i++) {
			digitalWrite(clockPinA, LOW);
			digitalWrite(clockPinA2, LOW);
			// digitalWrite(dataPinA, _numMatrix[0][i]);

			// Beat ones
			// digitalWrite(dataPinA, LOW);
			if (currentDig == 3) {
				digitalWrite(dataPinA, digitDisplay[6][i]);
				// digitalWrite(dataPinA, digitDisplay[state.beats % 10][i]);
			} else if (currentDig == 2) {
				digitalWrite(dataPinA, digitDisplay[6][i]);
				// digitalWrite(dataPinA, digitDisplay[state.beats / 10][i]);
			} else if (currentDig == 1) {
				digitalWrite(dataPinA, digitDisplay[6][i]);
				// digitalWrite(dataPinA, digitDisplay[state.div % 10][i]);
			} else {
				digitalWrite(dataPinA, digitDisplay[6][i]);
				// digitalWrite(dataPinA, digitDisplay[state.div / 10][i]);
			}
			digitalWrite(clockPinA, HIGH);
			digitalWrite(clockPinA2, HIGH);
		}

		digitalWrite(latchPinA, HIGH);
		digitalWrite(latchPinA2, HIGH);

		currentDig = (currentDig + 1) % 4;
		// currentDig = 0;

		ff = true;
	}

	return;

	// Serial.println(delta);
    
    // GPIO_read(module->getInputPinSchematic(), module->getInputBuffer(),
    //           module->getNumInputs());
    
    // GPIO_write(module->getOutputPinSchematic(), module->getOutputBuffer(),
    //            module->getNumOutputs());

	// Hardcoded stuff outside of GPIO_write

	// Read the encoder
	for (int i = 0; i < 8; i++) {
		digitalWrite(m0, bitRead(i, 0));
		digitalWrite(m1, bitRead(i, 1));
		digitalWrite(m2, bitRead(i, 2));

		analogVal = analogRead(analogMuxIn);
		digitalVal = digitalRead(digitalMuxIn);

		// Serial.print(i);
		// Serial.print(": ");
		// Serial.print(analogVal);
		// Serial.print(i == 7 ? "\n" : "\t");

		// continue;

		if (i == AnalogMux.DIVIDE_ATV) {
			pot0 = analogVal;
			// Serial.print("pot 1: ");
			// Serial.print(pot0);
			// Serial.print("\t");
		} else if (i == AnalogMux.TRUNCATE_ATV) {
			pot1 = analogVal;
			// Serial.print("pot 2: ");
			// Serial.print(pot1);
			// Serial.print("\t");
		}

		if (i == AnalogMux.LATCH_SWITCH) {
			// Serial.print("Latch switch: ");
			// Serial.print(analogVal);
			// Serial.print("\t");
		} else if (i == AnalogMux.ROUND_SWITCH) {
			// Serial.print("Round switch: ");
			// Serial.println(analogVal);
			// Serial.print("\n");
		}

		if (i == DigitalMux.DIVIDE_ENC_A) {
			divEncA = digitalVal ? 1 : 0;
			// Serial.print("divide-a: ");
			// Serial.print(divEncA);
			// Serial.print("\t");
		} else if (i == DigitalMux.DIVIDE_ENC_B) {
			divEncB = digitalVal ? 1 : 0;
			// Serial.print("divide-b: ");
			// Serial.print(divEncB);
			// Serial.print("\t");
		} else if (i == DigitalMux.BEAT_ENC_A) {
			beatsEncA = digitalVal ? 1 : 0;
			// Serial.print("beats-a: ");
			// Serial.print(beatsEncA);
			// Serial.print("\t");
		} else if (i == DigitalMux.BEAT_ENC_B) {
			beatsEncB = digitalVal ? 1 : 0;
			// Serial.print("beats-b: ");
			// Serial.print(beatsEncB);
			// Serial.print("\n");
		} else if (i == DigitalMux.BEAT_SWITCH) {
			if (digitalVal && (beat_switch_state_prev == 0)) {
				beat_latch = !beat_latch;
			}
			beat_switch_state_prev = digitalVal;
		} else if (i == DigitalMux.DIV_SWITCH) {
			if (digitalVal && (div_switch_state_prev == 0)) {
				div_latch = !div_latch;
			}
			div_switch_state_prev = digitalVal;
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
	ins[DEBUG_TEMPO] = (pot0 / 1024.0) * 220 + 20;

	// Process the module
	// module->process(delta);

	// Serial.print("beats: ");
	// Serial.print(state.beats);
	// Serial.print("\t");
	// Serial.print("div: ");
	// Serial.print(state.div);
	// Serial.print("\n");

	// Write the output pins
	double *ob = module->getOutputBuffer();
	digitalWrite(latchPinB, LOW);
	for (int i = 0; i < 8; i++) {
		int revPin = 7 - i;
		digitalWrite(clockPinB, LOW);

		if (revPin == ShiftOutputsB.TRUNCATE_LED) {
			digitalWrite(dataPinB, ob[PHASE_OUT] < 0.5 ? HIGH : LOW);
			// digitalWrite(dataPinB, (divEncA) ? HIGH : LOW);
		} else if (revPin == ShiftOutputsB.TRUNCATE_OUTPUT) {
			digitalWrite(dataPinB, HIGH);
			// digitalWrite(dataPinB, ob[PHASE_OUT] < 0.5 ? LOW : HIGH);
		} else if (revPin == ShiftOutputsB.DOWNBEAT_OUTPUT) {
			// digitalWrite(dataPinB, ob[DOWNBEAT_OUT] < 0.5 ? HIGH : LOW);
			digitalWrite(dataPinB, HIGH);
		} else if (revPin == ShiftOutputsB.BEAT_OUTPUT) {
			digitalWrite(dataPinB, HIGH);
			// digitalWrite(dataPinB, ob[BEATS_OUT] < 0.5 ? LOW : HIGH);
		} else if (revPin == ShiftOutputsB.EoM_OUTPUT) {
			digitalWrite(dataPinB, HIGH); // TODO: Add EoM
		} else if (revPin == ShiftOutputsB.DIVIDE_LED) {
			digitalWrite(dataPinB, ob[SUBDIVISIONS_OUT] < 0.5 ? HIGH : LOW);
			// digitalWrite(dataPinB, (beatsEncA) ? HIGH : LOW);
		} else if (revPin == ShiftOutputsB.DIVIDE_OUTPUT) {
			digitalWrite(dataPinB, HIGH);
			// digitalWrite(dataPinB, ob[SUBDIVISIONS_OUT] < 0.5 ? LOW : HIGH);
		} else if (revPin == ShiftOutputsB.NOTHING) {
			// skip
		}

		digitalWrite(clockPinB, HIGH);
	}
	digitalWrite(latchPinB, HIGH);

	// Write the output pins
	digitalWrite(latchPinC, LOW);
	for (int i = 0; i < 8; i++) {
		int revPin = 7 - i;
		digitalWrite(clockPinC, LOW);

		if (revPin == ShiftOutputsC.MOD_LED) {
			digitalWrite(dataPinC, LOW); // TODO: Add modulation to outputs
		} else if (revPin == ShiftOutputsC.EoM_LED) {
			digitalWrite(dataPinC, LOW); // TODO: Add EoM to outputs
		} else if (revPin == ShiftOutputsC.CLOCK_LED) {
			digitalWrite(dataPinC, HIGH); // TODO: Add Clock to outputs
		} else if (revPin == ShiftOutputsC.DOWNBEAT_LED) {
			digitalWrite(dataPinC, ob[DOWNBEAT_OUT] < 0.5 ? HIGH : LOW);
		} else if (revPin == ShiftOutputsC.BEAT_LED) {
			// Serial.println(ob[BEATS_OUT]);
			digitalWrite(dataPinC, ob[BEATS_OUT] < 0.5 ? HIGH : LOW);
		} else if (revPin == ShiftOutputsC.BEAT_LATCH_LED) {
			digitalWrite(dataPinC, beat_latch ? HIGH : LOW); // TODO: Add beat latch to outputs
		} else if (revPin == ShiftOutputsC.DIV_LATCH_LED) {
			digitalWrite(dataPinC, div_latch ? HIGH : LOW); // TODO: Add div latch to outputs
		} else if (revPin == ShiftOutputsC.NOTHING) {
			// skip
		}

		// digitalWrite(dataPinC, LOW);

		digitalWrite(clockPinC, HIGH);
	}
	digitalWrite(latchPinC, HIGH);

	lastMillis = thisMillis;
}
