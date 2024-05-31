#include "mock.h"
#include "../src/peripherals/pump.cpp"
#include "utils/test-utils.h"

void test_pump_clicks_for_flow_correct_binary_search(void) {
  //corrected from 31 old value due to correcting the formula for flow/click calculations
  TEST_ASSERT_EQUAL_FLOAT_ACCURACY(24, getLoadForFlow(2, 5) * maxPumpClicksPerSecond, 0);
  TEST_ASSERT_EQUAL_FLOAT_ACCURACY(50, getLoadForFlow(9, 10) * maxPumpClicksPerSecond, 0);
  TEST_ASSERT_EQUAL_FLOAT_ACCURACY(0, getLoadForFlow(0, 0) * maxPumpClicksPerSecond, 0);
}

void test_pump_get_flow(void) {
  TEST_ASSERT_EQUAL_FLOAT_ACCURACY(5.f, getPumpFlow(2, getLoadForFlow(2, 5) * maxPumpClicksPerSecond), 1);
  TEST_ASSERT_EQUAL_FLOAT_ACCURACY(4.5f, getPumpFlow(5, getLoadForFlow(5, 4.5) * maxPumpClicksPerSecond), 1);
}

void runAllPumpTests() {
  RUN_TEST(test_pump_clicks_for_flow_correct_binary_search);
  RUN_TEST(test_pump_get_flow);
}
