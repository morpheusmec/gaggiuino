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

void deScale(eepromValues_t &runningCfg, const SensorState &currentState) {
  switch (descalingState) {
    case DescalingState::IDLE: // Waiting for fuckfest to begin
      if (currentState.brewSwitchState) {
        ACTIVE_PROFILE(runningCfg).setpoint = 9;
        openValve();
        setSteamValveRelayOn();
        descalingState = DescalingState::DESCALING_PHASE1;
        descalingCycle = 0;
        descalingTimer = millis();
      }
      break;
    case DescalingState::DESCALING_PHASE1: // Slowly penetrating that scale
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setPumpToPercentage(0.1);
      if (millis() - descalingTimer > DESCALE_PHASE1_EVERY) {
        lcdSetDescaleCycle(descalingCycle++);
        if (descalingCycle < 100) {
          descalingTimer = millis();
          descalingState = DescalingState::DESCALING_PHASE2;
        } else {
          descalingState = DescalingState::FINISHED;
        }
      }
      break;
    case DescalingState::DESCALING_PHASE2: // Softening the f outta that scale
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setPumpOff();
      if (millis() - descalingTimer > DESCALE_PHASE2_EVERY) {
        descalingTimer = millis();
        lcdSetDescaleCycle(descalingCycle++);
        descalingState = DescalingState::DESCALING_PHASE3;
      }
      break;
    case DescalingState::DESCALING_PHASE3: // Fucking up that scale big time
      currentState.brewSwitchState ? descalingState : descalingState = DescalingState::FINISHED;
      setPumpToPercentage(0.30);
      if (millis() - descalingTimer > DESCALE_PHASE3_EVERY) {
        solenoidBeat();
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
      closeValve();
      setSteamValveRelayOff();
      currentState.brewSwitchState ? descalingState = DescalingState::FINISHED : descalingState = DescalingState::IDLE;
      if (millis() - descalingTimer > 1000) {
        lcdBrewTimerStop();
        lcdShowPopup("FINISHED");
        descalingTimer = millis();
      }
      break;
  }
  justDoCoffee(runningCfg, currentState);
}

void solenoidBeat() {
  setPumpFullOn();
  closeValve();
  delay(1000);
  watchdogReload();
  openValve();
  delay(200);
  closeValve();
  delay(1000);
  watchdogReload();
  openValve();
  delay(200);
  closeValve();
  delay(1000);
  watchdogReload();
  openValve();
  setPumpOff();
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
