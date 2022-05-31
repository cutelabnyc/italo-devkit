#include <Arduino.h>

// Shift register pins (seven segment)
int dataPinA = 11; //PB3
int latchPinA = 9; //PB1
int clockPinA = 10; //PB2

// const int digitDisplay[10][8] {
// 	// NUMBERS
// 	{1,0,1,1,1,1,1,0}, //ZERO
// 	{0,0,0,0,0,1,1,0}, //ONE
// 	{1,1,0,1,1,0,1,0}, //TWO
// 	{1,1,0,0,1,1,1,0}, //THREE
// 	{0,1,1,0,0,1,1,0}, //FOUR
// 	{1,1,1,0,1,1,0,0}, //FIVE
// 	{1,1,1,1,1,1,0,0}, //SIX
// 	{1,0,0,0,0,1,1,0}, //SEVEN
// 	{1,1,1,1,1,1,1,1}, //EIGHT
// 	{1,1,1,0,1,1,1,0}, //NINE
// };

// const int digitDisplay[10][8] {
// 	// NUMBERS
// 	{1,0,0,0,0,0,0,0}, //ZERO
// 	{0,1,0,0,0,0,0,0}, //ONE
// 	{0,0,1,0,0,0,0,0}, //TWO
// 	{0,0,0,1,0,0,0,0}, //THREE
// 	{0,0,0,0,1,0,0,0}, //FOUR
// 	{0,0,0,0,0,1,0,0}, //FIVE
// 	{0,0,0,0,0,0,1,0}, //SIX
// 	{0,0,0,0,0,0,0,1}, //SEVEN
// 	{0,0,0,0,0,0,0,0}, //EIGHT
// 	{0,0,0,0,0,0,0,0}, //NINE
// };

const int digitDisplay[10][8] {
	// NUMBERS
	{1,1,1,1,1,0,1,0}, //ZERO
	{0,0,1,0,0,0,1,0}, //ONE
	{1,1,0,1,0,1,1,0}, //TWO
	{0,1,1,1,0,1,1,0}, //THREE
	{0,0,1,0,1,1,1,0}, //FOUR
	{0,1,1,1,1,1,0,0}, //FIVE
	{1,1,1,1,1,1,0,0}, //SIX
	{0,0,1,1,0,0,1,0}, //SEVEN
	{1,1,1,1,1,1,1,0}, //EIGHT
	{0,0,1,1,1,1,1,0}, //NINE
};

void setup()
{   
    Serial.begin(9600);

	Serial.println("setup");

	// Setting all of the shift register pins to be outputs
	pinMode(clockPinA, OUTPUT);
	pinMode(latchPinA, OUTPUT);
	pinMode(dataPinA, OUTPUT);
}

int hasWrittenDisplay = 0;
int currentDigit = 0;

/**
 * The three step process consists of reading GPIO values,
 * processing the data, and writing the output values.
 **/
void loop()
{


	// Write the display
	// if (!hasWrittenDisplay) {
		// They all share a port so this should be fine
		uint8_t port = digitalPinToPort(latchPinA);
		volatile uint8_t *out;

		// Serial.println("Writing the display");

		// Start by writing both latch pins low. Shift registers don't update
		// until the latch pin goes high
		digitalWrite(latchPinA, LOW);
		// digitalWrite(latchPinA2, LOW);

		// This should write both latch pins low at the same time
		// uint8_t latchBitA = digitalPinToBitMask(latchPinA);
		// uint8_t latchBitB = digitalPinToBitMask(latchPinA2);
		// uint8_t mask = ~latchBitA & ~latchBitB;
		// out = portOutputRegister(port);
		// *out &= mask;

		// Enable the digit of interest
		for (int i = 7; i >= 0; i--) {
			digitalWrite(clockPinA, LOW);
			// digitalWrite(clockPinA2, LOW);

			digitalWrite(dataPinA, i == currentDigit ? HIGH : LOW);
			// digitalWrite(dataPinA, HIGH);

			digitalWrite(clockPinA, HIGH);
			// digitalWrite(clockPinA2, HIGH);
		}

		// Write the digit
		for (int i = 7; i >= 0; i--) {
			digitalWrite(clockPinA, LOW);
			// digitalWrite(clockPinA2, LOW);
			
			digitalWrite(dataPinA, digitDisplay[(currentDigit + 1) % 10][i]);

			digitalWrite(clockPinA, HIGH);
			// digitalWrite(clockPinA2, HIGH);
		}

		// Turn everything on but for real
		// for (int i = 0; i < 24; i++) {
		// 	digitalWrite(clockPinA, LOW);
		// 	digitalWrite(clockPinA2, LOW);

		// 	// digitalWrite(dataPinA, i == currentDigit ? HIGH : LOW);
		// 	digitalWrite(dataPinA, HIGH);
			
		// 	digitalWrite(clockPinA, HIGH);
		// 	digitalWrite(clockPinA2, HIGH);
		// }

		// Write exactly our chosen 16 bits

		// All on
		// const int fixedBits[16] = {
		// 	1, 1, 1, 1,
		// 	1, 1, 1, 1,
		// 	1, 1, 1, 1,
		// 	1, 1, 1, 1
		// };

		// //	All off
		// const int fixedBits[16] = {
		// 	0, 0, 0, 0,
		// 	0, 0, 0, 0,
		// 	0, 0, 0, 0,
		// 	0, 0, 0, 0
		// };

		// Write exactly our chosen 16 bits
		// const int fixedBits[16] = {
		// 	0, 0, 0, 0,
		// 	0, 0, 0, 1,
		// 	0, 0, 0, 0,
		// 	0, 0, 1, 0,
		// };

		// for (int i = 0; i < 16; i++) {
		// 	digitalWrite(clockPinA, LOW);
		// 	// digitalWrite(clockPinA2, LOW);
		// 	// Write both clock pins low at the same time
		// 	// uint8_t clockBitA = digitalPinToBitMask(clockPinA);
		// 	// uint8_t clockBitB = digitalPinToBitMask(clockPinA2);
		// 	// mask = ~clockBitA & ~clockBitB;
		// 	// out = portOutputRegister(port);
		// 	// *out &= mask;

		// 	// digitalWrite(dataPinA, i == currentDigit ? HIGH : LOW);
		// 	digitalWrite(dataPinA, fixedBits[i] ? HIGH : LOW);

		// 	digitalWrite(clockPinA, HIGH);
		// 	// digitalWrite(clockPinA2, HIGH);
		// 	// Write both clock pins high at the same time
		// 	// clockBitA = digitalPinToBitMask(clockPinA);
		// 	// clockBitB = digitalPinToBitMask(clockPinA2);
		// 	// mask = clockBitA | clockBitB;
		// 	// out = portOutputRegister(port);
		// 	// *out |= mask;
		// }

		digitalWrite(latchPinA, HIGH);
		// digitalWrite(latchPinA2, HIGH);

		// Write both latch pins high at the same time
		// latchBitA = digitalPinToBitMask(latchPinA);
		// latchBitB = digitalPinToBitMask(latchPinA2);
		// mask = latchBitA | latchBitB;
		// out = portOutputRegister(port);
		// *out |= mask;

		currentDigit = (currentDigit + 1) % 4;
		// Serial.println(currentDigit);

	// 	hasWrittenDisplay = true;
	// }
}
