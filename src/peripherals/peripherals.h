/* 09:32 15/03/2023 - change triggering comment */
#ifndef PERIPHERALS_H
#define PERIPHERALS_H

#include "pindef.h"
#include "peripherals.h"
#include <Arduino.h>

static inline void pinInit(void) {
  #if defined(LEGO_VALVE_RELAY)
    pinMode(valvePin, OUTPUT_OPEN_DRAIN);
  #else
    pinMode(valvePin, OUTPUT);
  #endif
  pinMode(relayPin, OUTPUT);
  #ifdef steamValveRelayPin
  pinMode(steamValveRelayPin, OUTPUT);
  #endif
  #ifdef steamBoilerRelayPin
  pinMode(steamBoilerRelayPin, OUTPUT);
  #endif
  pinMode(cup1DtcPin,  INPUT_PULLUP);
  pinMode(cup2DtcPin,  INPUT_PULLUP);
  pinMode(sol2Pin,  OUTPUT_OPEN_DRAIN);
  pinMode(sol3Pin,  OUTPUT_OPEN_DRAIN);
  pinMode(steamPin, INPUT_PULLUP);
  #ifdef waterPin
  pinMode(waterPin, INPUT_PULLUP);
  #endif
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

static inline void setSteamValveRelayOn(void) {
  #ifdef steamValveRelayPin
  digitalWrite(steamValveRelayPin, HIGH);  // steamValveRelayPin -> HIGH
  #endif
}

static inline void setSteamValveRelayOff(void) {
  #ifdef steamValveRelayPin
  digitalWrite(steamValveRelayPin, LOW);  // steamValveRelayPin -> LOW
  #endif
}

static inline void setSteamBoilerRelayOn(void) {
  #ifdef steamBoilerRelayPin
  digitalWrite(steamBoilerRelayPin, HIGH);  // steamBoilerRelayPin -> HIGH
  #endif
}

static inline void setSteamBoilerRelayOff(void) {
  #ifdef steamBoilerRelayPin
  digitalWrite(steamBoilerRelayPin, LOW);  // steamBoilerRelayPin -> LOW
  #endif
}



// Returns HIGH when switch is OFF and LOW when ON
// pin will be high when switch is ON.
static inline bool steamState(void) {
  return digitalRead(steamPin) == LOW; // pin will be low when switch is ON.
}

static inline bool waterPinState(void) {
  #ifdef waterPin
  return digitalRead(waterPin) == LOW; // pin will be low when switch is ON.
  #else
  return false;
  #endif
}

static inline void openValve(void) {
  #if defined LEGO_VALVE_RELAY
    digitalWrite(valvePin, LOW);
      #else
    digitalWrite(valvePin, HIGH);
  #endif
}

static inline void closeValve(void) {
  #if defined LEGO_VALVE_RELAY
    digitalWrite(valvePin, HIGH);
      #else
    digitalWrite(valvePin, LOW);
  #endif
}

#endif
