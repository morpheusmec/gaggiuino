/* 09:32 15/03/2023 - change triggering comment */
#ifndef JUST_DO_COFFEE_H
#define JUST_DO_COFFEE_H

#include "utils.h"
#include "../peripherals/peripherals.h"
#include "../peripherals/pump.h"
#include "../eeprom_data/eeprom_data.h"
#include "../log.h"
#include "sensors_state.h"
#include <Arduino.h>
#include "PIDController.h"


const uint32_t STEAM_TIMEOUT = 180000UL; // steam mode timeout
const float MAX_WATER_TEMP = 95.f; // max temp allowed in hot water mode
const float steamThreshold_ = 11.f; // max allowed pressure before heaters are disabled
const float activeSteamPressure_ = 2.f; // DreamSteam enabling threshold.
const float passiveSteamPressure_ = 3.f; // DreamSteam disabling threshold.

enum class HEATING {
  MODE_brew,
  MODE_steam,
  MODE_hotWater
};

void justDoCoffee(const GaggiaSettings &settings, SensorState &currentState);
void setHeatersPower(const float powerFactor);
void setBoilerOn();
void setBoilerOff();
void steamCtrl(const GaggiaSettings &settings, SensorState &currentState);
void hotWaterMode(SensorState &currentState);
void resetHeating(void);

#endif

