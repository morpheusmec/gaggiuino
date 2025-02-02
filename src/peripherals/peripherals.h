/* 09:32 15/03/2023 - change triggering comment */
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "pindef.h"
#include "peripherals.h"
#include <Arduino.h>

static inline void pinInit(void) {
  pinMode(cup1DtcPin,  INPUT_PULLUP);
  pinMode(cup2DtcPin,  INPUT_PULLUP);
  pinMode(steamPin, INPUT_PULLUP);
  pinMode(waterPin, INPUT_PULLUP);

digitalWrite(sol2Pin, HIGH);
  pinMode(sol2Pin,  OUTPUT_OPEN_DRAIN);
digitalWrite(sol3Pin, HIGH);
  pinMode(sol3Pin,  OUTPUT_OPEN_DRAIN);

  pinMode(thermoRDY, INPUT_PULLUP);
  digitalWrite(shutdownPin, HIGH);
  pinMode(shutdownPin, OUTPUT_OPEN_DRAIN);
}

static inline bool tempReady(void) {
  return digitalRead(thermoRDY) == LOW;
}

static inline bool cup1BtnState(void) {
  return digitalRead(cup1DtcPin) == LOW;
}

static inline bool cup2BtnState(void) {
  return digitalRead(cup2DtcPin) == LOW;
}

static inline void setSol2On(void) {
  digitalWrite(sol2Pin, LOW); 
}

static inline void setSol2Off(void) {
  digitalWrite(sol2Pin, HIGH); 
}

static inline void setSol3On(void) {
  digitalWrite(sol3Pin, LOW); 
}

static inline void setSol3Off(void) {
  digitalWrite(sol3Pin, HIGH); 
}


// Returns HIGH when switch is OFF and LOW when ON
// pin will be high when switch is ON.
static inline bool steamBtnState(void) {
  return digitalRead(steamPin) == LOW; // pin will be low when switch is ON.
}

static inline bool waterBtnState(void) {
  return digitalRead(waterPin) == LOW; // pin will be low when switch is ON.
}

#endif
