#include <interfaces.hpp>
#include <pass-thru.h>
#include <stdio.h>
#include <unity.h>

passthru_t pt;

void setUp(void) {
  // set stuff up here
  ModuleInterface<double> *module = buildModule();
  module->init();
}

void tearDown(void) {
  // clean stuff up here
}

void test_pt(void) {
  module->readParameters()
      TEST_ASSERT_EQUAL_UINT16(in_data[s][c], out_data[s][c]);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_pt);
  UNITY_END();

  return 0;
}
