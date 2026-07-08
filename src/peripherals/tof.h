#ifndef TOF_H
#define TOF_H

#include "HardwareTimer.h"
#include <stdint.h> // for uint8_t
#include <Adafruit_VL53L0X.h>
#include <movingAvg.h>
#include "system_state.h"
#include <algorithm> // for std::generate

Adafruit_VL53L0X tof_sensor;
movingAvg mvAvg(4);

class TOF {
  public:
    TOF();
    void init(SystemState&);
    void setCustomRanges(uint16_t startValue, uint16_t endValue);
    uint16_t readLvl();
    uint16_t readRangeToPct(uint16_t);

  private:
    uint32_t tofReading;
    uint16_t tofStartValue = 50u; // mm
    uint16_t tofEndValue = 200u; // mm
};

TOF::TOF() {}

void TOF::init(SystemState& systemState) {
  #ifdef TOF_VL53L0X
  while(!systemState.tofReady) {
    systemState.tofReady = tof_sensor.begin(0x29, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  }
  tof_sensor.startRangeContinuous();
  mvAvg.begin();
  #endif
}

void TOF::setCustomRanges(uint16_t startValue, uint16_t endValue) {
  TOF::tofStartValue = startValue;
  TOF::tofEndValue = endValue;
}

uint16_t TOF::readLvl() {
  #ifdef TOF_VL53L0X
  if(tof_sensor.isRangeComplete()) {
    TOF::tofReading = mvAvg.reading(tof_sensor.readRangeResult());
  }
  #endif
  return  TOF::tofReading != 0 ? readRangeToPct(TOF::tofReading) : 30u;
}

uint16_t TOF::readRangeToPct(uint16_t val) {
  return (uint16_t) constrain(map(val * 0.792 + 13.5, TOF::tofStartValue, TOF::tofEndValue, 100, 0), 0, 100);
}

#endif
