/* 09:32 15/03/2023 - change triggering comment */
#ifndef GAGGIUINO_H
#define GAGGIUINO_H

#include <Arduino.h>
#include <SimpleKalmanFilter.h>

#include "log.h"
#include "peripherals/front_panel_leds.h"
#include "peripherals/internal_watchdog.h"
#include "peripherals/pump.h"
#include "peripherals/pressure_sensor.h"
#include "peripherals/scales.h"
#include "peripherals/peripherals.h"
#include "peripherals/thermocouple.h"
#include "sensors_state.h"
#include "system_state.h"
#include "functional/descale.h"
#include "functional/just_do_coffee.h"
#include "functional/predictive_weight.h"
#include "functional/shot_profiler.h"
#include "profiling_phases.h"
#include "peripherals/esp_comms.h"
#include "peripherals/tof.h"

// Define some const values
#if defined SINGLE_BOARD
    #define GET_KTYPE_READ_EVERY    70 // max31855 amp module data read interval not recommended to be changed to lower than 70 (ms)
#else
    #define GET_KTYPE_READ_EVERY    250 // max6675 amp module data read interval not recommended to be changed to lower than 250 (ms)
#endif
#define GET_PRESSURE_READ_EVERY 10 // Pressure refresh interval (ms)
#define GET_SCALES_READ_EVERY   100 // Scales refresh interval (ms)
#define GET_SCALES_ACCIDENTAL   2000u // Accidental touches or placing cup on scales post brew activation timeout
#define REFRESH_ESP_DATA_EVERY  100 // Screen refresh interval (ms)
#define REFRESH_SCREEN_EVERY    150 // Screen refresh interval (ms)
#define REFRESH_FLOW_EVERY      100 // Flow refresh interval (ms)
#define HEALTHCHECK_EVERY       30000 // System checks happen every 30sec

enum class OPERATION_MODES {
  OPMODE_straight9Bar,
  OPMODE_justPreinfusion,
  OPMODE_justPressureProfile,
  OPMODE_manual,
  OPMODE_preinfusionAndPressureProfile,
  OPMODE_flush,
  OPMODE_descale,
  OPMODE_flowPreinfusionStraight9BarProfiling,
  OPMODE_justFlowBasedProfiling,
  OPMODE_steam,
  OPMODE_FlowBasedPreinfusionPressureBasedProfiling,
  OPMODE_everythingFlowProfiled,
  OPMODE_pressureBasedPreinfusionAndFlowProfile
};

#if not defined(TOF_START) || not defined(TOF_END)
#define TOF_START 40u
#define TOF_END 165u
#endif

const uint16_t tofStartValue = TOF_START; // Tof offset when tank is full
const uint16_t tofEndValue = TOF_END; // Tof offset when tank is nearly empty
const float weightRateThreshold = 9.f; // The rate of weigh random change(aka accidental scales touching)
const float weightIncreaseThreshold = 40.f; // Accounting for placing a cup on the scales after initiating brew

//Timers
unsigned long systemHealthTimer;
unsigned long pageRefreshTimer;
unsigned long pressureTimer;
unsigned long brewingTimer;
unsigned long thermoTimer;
unsigned long scalesTimer;
unsigned long scalesTimeout;
unsigned long flowTimer;
unsigned long iddleTimer;

//scales vars
Measurements weightMeasurements(4);


//PP&PI variables
int preInfusionFinishedPhaseIdx = 3;

// Other util vars
float previousSmoothedPressure;
float previousSmoothedPumpFlow;

static void sysHealthCheck();

#endif
