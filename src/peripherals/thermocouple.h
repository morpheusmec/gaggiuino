/* 09:32 15/03/2023 - change triggering comment */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include "pindef.h"

#if defined SINGLE_BOARD
#include <Adafruit_MAX31855.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
Adafruit_MAX31855 thermocouple(thermoCS, &thermoSPI);
#else
#include <Adafruit_MAX31856.h>
SPIClass thermoSPI(thermoDI, thermoDO, thermoCLK);
Adafruit_MAX31856 thermocouple(thermoCS, &thermoSPI);
#endif

static inline void thermocoupleInit(void) {
  thermocouple.begin();
  thermocouple.setThermocoupleType(MAX31856_TCTYPE_E);
}

static inline float thermocoupleRead(void) {
  return thermocouple.readThermocoupleTemperature();
}

#endif
