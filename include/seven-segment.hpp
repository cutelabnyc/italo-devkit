#include "shift-register.hpp"
#include <Arduino.h>

enum class SpecialDigits {
    P = 10,
    Nothing = 11
};

const int digitDisplay[12][8]{
    // NUMBERS
    {1, 1, 1, 1, 1, 0, 1, 0}, // ZERO
    {0, 0, 1, 0, 0, 0, 1, 0}, // ONE
    {1, 1, 0, 1, 0, 1, 1, 0}, // TWO
    {0, 1, 1, 1, 0, 1, 1, 0}, // THREE
    {0, 0, 1, 0, 1, 1, 1, 0}, // FOUR
    {0, 1, 1, 1, 1, 1, 0, 0}, // FIVE
    {1, 1, 1, 1, 1, 1, 0, 0}, // SIX
    {0, 0, 1, 1, 0, 0, 1, 0}, // SEVEN
    {1, 1, 1, 1, 1, 1, 1, 0}, // EIGHT
    {0, 0, 1, 1, 1, 1, 1, 0}, // NINE

    // SPECIAL CHARACTERS
    {1, 0, 0, 1, 1, 1, 1, 0}, // P
    {0, 0, 0, 0, 0, 0, 0, 0}, // Nothing
};

static void seven_segment_process(shift_register_t *shift_register,
                                  uint8_t indexToWrite, uint8_t valueToWrite, uint8_t decimal) {

    uint8_t bitsToWrite[16];

    for (int i = 0; i < 8; i++) {
        bool writeHigh = (i == indexToWrite);
        writeHigh |= (i == 6) && decimal;
        bitsToWrite[7 - i] = (writeHigh ? HIGH : LOW);
        bitsToWrite[15 - i] = digitDisplay[valueToWrite][i];
    }

    shift_register_process(shift_register, bitsToWrite, 16, false);
}
