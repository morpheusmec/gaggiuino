/* 09:32 15/03/2023 - change triggering comment */
#ifndef PUMP_H
#define PUMP_H

#include <Arduino.h>
#include "sensors_state.h"

#define ZC_MODE    FALLING
#define between(x,a,b) ( b>a ? (x>=a)&&(x<=b) : (x>=b)&&(x<=a))

constexpr uint8_t PUMP_RANGE = 250;

float findQ(float p, float l);
float findL(float p, float q);
void pumpInit(const int powerLineFrequency, const float pumpFlowAtZero);
void setPumpPressure(const float targetPressure, const float flowRestriction, const SensorState &currentState);
void setPumpOff(void);
void setPumpFullOn(void);
float getCurrentPumpLoad(void);
void setPumpToPercentage(const float percentage);
void setHeaterToPercentage(const float percentage);
long  getAndResetClickCounter(void);
int getCPS(void);
void pumpPhaseShift(void);
void pumpStopAfter(const uint8_t val);
float getPumpFlow(const float pressure, const float cps);
// float getPumpFlowPerClick(const float pressure);
float getLoadForFlow(const float pressure, const float flow);
void setPumpFlow(const float targetFlow, const float pressureRestriction, const SensorState &currentState);
#endif
