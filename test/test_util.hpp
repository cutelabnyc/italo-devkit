#ifndef TEST_UTIL_H
#define TEST_UTIL_H

#include <stdint.h>
#include <unity.h>

/**
 * test_util.hpp
 *
 * This could potentially be the start of a library
 * of our own custom test utilities, just incase
 * Unity is ever lacking in some regard
 */

typedef void (*processor_t)(void *handle, uint16_t *in, uint16_t *out);

void run_equality_test(void *handle, processor_t process, uint16_t *ins,
                       uint16_t *outs, uint16_t *expected, uint8_t count);

#endif // TEST_UTIL_H
