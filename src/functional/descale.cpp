/* 09:32 15/03/2023 - change triggering comment */
#include "descale.h"
#include "just_do_coffee.h"
#include "../peripherals/internal_watchdog.h"
#include "../lcd/lcd.h"

DescalingState descalingState = DescalingState::IDLE;

short flushCounter = 0;
uint8_t counter = 0;
unsigned long descalingTimer = 0;
int descalingCycle = 0;

void deScale(GaggiaSettings &settings, SensorState &currentState) {
  float pumpSpeed = 1./3.;
  uint16_t time_each = 10000;
  switch (descalingState) {
    case DescalingState::IDLE: // Waiting for fuckfest to begin
      if (currentState.brewSwitchState) {
        descalingState = DescalingState::DESCALING_PHASE1;
        descalingCycle = 0;
        descalingTimer = millis();
      }
      break;
    case DescalingState::DESCALING_PHASE1: // Normally open
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setSol2Off();
      setSol3Off();
      if (!currentState.hotWaterSwitchState && !currentState.steamSwitchState){
        setPumpToPercentage(pumpSpeed);
      } else {
        setPumpOff();
        lcdShowPopup("Turn knob to neutral position");
        descalingTimer = millis();
      }
      if (millis() - descalingTimer > 3000) {
        lcdSetDescaleCycle(descalingCycle++);
        if (descalingCycle < 100) {
          descalingTimer = millis();
          descalingState = DescalingState::DESCALING_PHASE2;
        } else {
          descalingState = DescalingState::FINISHED;
        }
      }
      break;
    case DescalingState::DESCALING_PHASE2: // hot water
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setSol2Off();
      setSol3Off();
      if (currentState.hotWaterSwitchState){
        setPumpToPercentage(pumpSpeed);
      } else {
        setPumpOff();
        lcdShowPopup("Turn knob to hot water position");
        descalingTimer = millis();
      }
      if (millis() - descalingTimer > time_each) {
        descalingTimer = millis();
        lcdSetDescaleCycle(descalingCycle++);
        descalingState = DescalingState::DESCALING_PHASE3;
      }
      break;
    case DescalingState::DESCALING_PHASE3: // steam wand
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setSol2Off();
      setSol3Off();
      if (currentState.steamSwitchState){
        setPumpToPercentage(pumpSpeed);
      } else {
        setPumpOff();
        lcdShowPopup("Turn knob to steam position");
        descalingTimer = millis();
      }
      if (millis() - descalingTimer > time_each) {
        solenoidBeat3W();
        lcdSetDescaleCycle(descalingCycle++);
        if (descalingCycle < 100) {
          descalingTimer = millis();
          descalingState = DescalingState::DESCALING_PHASE4;
        } else {
          descalingState = DescalingState::FINISHED;
        }
      }
      break;
    case DescalingState::DESCALING_PHASE4: // Brewhead
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setSol2Off();
      setSol3On();
      setPumpToPercentage(pumpSpeed);
      if (millis() - descalingTimer > time_each) {
        lcdSetDescaleCycle(descalingCycle++);
        if (descalingCycle < 100) {
          descalingTimer = millis();
          descalingState = DescalingState::DESCALING_PHASE5;
        } else {
          descalingState = DescalingState::FINISHED;
        }
      }
      break;
    case DescalingState::DESCALING_PHASE5: // purge
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setSol2On();
      setSol3On();
      setPumpToPercentage(pumpSpeed);
      if (millis() - descalingTimer > 3000) {
        solenoidBeat2W();
        lcdSetDescaleCycle(descalingCycle++);
        if (descalingCycle < 100) {
          descalingTimer = millis();
          descalingState = DescalingState::DESCALING_PHASE1;
        } else {
          descalingState = DescalingState::FINISHED;
        }
      }
      break;
    case DescalingState::FINISHED: // Scale successufuly fucked
      setPumpOff();
      setSol2Off();
      setSol3Off();
      currentState.brewSwitchState ? descalingState = DescalingState::FINISHED : descalingState = DescalingState::IDLE;
      currentState.brewSwitchState = false;
      if (millis() - descalingTimer > 1000) {
        lcdBrewTimerStop();
        lcdShowPopup("FINISHED");
        descalingTimer = millis();
      }
      break;
  }
  justDoCoffee(settings, currentState, 70.f);
}

void solenoidBeat3W() {
  setPumpFullOn();
  setSol3On();
  delay(1000);
  watchdogReload();
  setSol3Off();
  delay(200);
  setSol3On();
  delay(1000);
  watchdogReload();
  setSol3Off();
  delay(200);
  setSol3On();
  delay(1000);
  watchdogReload();
  setSol3Off();
  setPumpOff();
}

void solenoidBeat2W() {
  setSol3On();
  setPumpFullOn();
  setSol2On();
  delay(1000);
  watchdogReload();
  setSol2Off();
  delay(200);
  setSol2On();
  delay(1000);
  watchdogReload();
  setSol2Off();
  delay(200);
  setSol2On();
  delay(1000);
  watchdogReload();
  setSol2Off();
  setPumpOff();
  setSol3Off();
}

void backFlush(SensorState &currentState) {
  if (currentState.brewSwitchState || currentState.flushSwitchState) {
    if (flushCounter >= 2 * BACK_FLUSH_CYCLES ) {
      currentState.brewSwitchState = false;
      currentState.flushSwitchState = false;
    }
    else {
      setSol3On();
      flushPhases();
    } 
  } else {
    flushDeactivated();
    flushCounter = 0;
  }
}

void flushActivated(void) {
  setSol3On();
  setSol2Off();
  setPumpToPercentage(0.5f);
}

void flushDeactivated(void) {
  setSol2Off();
  setSol3Off();
  setPumpOff();
}

void flushPhases(void) {
  static long timer = millis();
  if (flushCounter < 2 * BACK_FLUSH_CYCLES) {
    if (flushCounter % 2 == 0) {
      if (millis() - timer >= 15000) {
        flushCounter++;
        timer = millis();
      }
      setSol2Off();
      setPumpToPercentage(0.5f);
    } else {
      if (millis() - timer >= 15000) {
        flushCounter++;
        timer = millis();
      }
      setSol2On();
      ((flushCounter == 2 * BACK_FLUSH_CYCLES - 1) && (millis() - timer > 5000) && (millis() - timer < 10000)) ? setPumpToPercentage(0.5) : setPumpOff();
    }
  } else {
    flushDeactivated();
    timer = millis();
  }
}
