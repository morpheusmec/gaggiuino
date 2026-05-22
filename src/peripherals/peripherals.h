/* 09:32 15/03/2023 - change triggering comment */
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "pindef.h"
#include "peripherals.h"
#include <Arduino.h>
#include <PCF8575.h>

extern PCF8575 PCF;

static inline void pinInit(void) {
  PCF.begin();

  pinMode(thermoRDY, INPUT_PULLUP);
  digitalWrite(shutdownPin, HIGH);
  pinMode(shutdownPin, OUTPUT_OPEN_DRAIN);
}

static inline bool tempReady(void) {
  return digitalRead(thermoRDY) == LOW;
}

static inline void readPCF(void){
  PCF.read16();
}

static inline bool cup1BtnState(void) {
  return (PCF.value() & (1 << cup1Btn)) == LOW;
}

static inline bool cup2BtnState(void) {
  return (PCF.value() & (1 << cup2Btn)) == LOW;
}

static inline bool steamBtnState(void) {
  return (PCF.value() & (1 << steamBtn)) == LOW;
}

static inline bool waterBtnState(void) {
  return (PCF.value() & (1 << hotWaterBtn)) == LOW;
}
  
static inline void setSol2On(void) {
  PCF.write(sol2Pin, LOW);
}

static inline void setSol2Off(void) {
  PCF.write(sol2Pin, HIGH);
}

static inline void setSol3On(void) {
  PCF.write(sol3Pin, LOW);
}

static inline void setSol3Off(void) {
  PCF.write(sol3Pin, HIGH);
}

#endif
