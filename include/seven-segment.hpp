#include "shift-register.hpp"
#include <Arduino.h>

const int digitDisplay[10][8]{
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
};

static void seven_segment_process(shift_register_t *shift_register,
                                  uint8_t indexToWrite, uint8_t valueToWrite) {

  uint8_t bitsToWrite[16];

  for (int i = 0; i < 8; i++) {
    bitsToWrite[7 - i] = (indexToWrite == i ? HIGH : LOW);
    bitsToWrite[15 - i] = digitDisplay[valueToWrite][i];
  }

  shift_register_process(shift_register, bitsToWrite, 16);
}
