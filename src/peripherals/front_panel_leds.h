#ifndef FRONT_PANEL_LEDS_H
#define FRONT_PANEL_LEDS_H

#include "peripherals.h"
#include "PCF8575.h"

enum class FrontLedsState {
  LEDS_OFF = 0,
  IDDLE = 1,
  STARTUP = 2,
  BREWING = 3,
  FLUSHING = 4,
  HOTWATER = 5,
  STEAM_HEATING = 6,
  STEAM_READY = 7,
  STEAMING = 8,
  STEAM_FLUSHING = 9,
};

class FrontPanelLeds{
  private:
  PCF8575& PCF;
  FrontLedsState currentState = FrontLedsState::LEDS_OFF;
  FrontLedsState previousState = FrontLedsState::LEDS_OFF;
  uint16_t ledMask = 0;
  uint16_t dataOut = 0;
  uint32_t blinkTimer = 0;
  

  uint16_t getDataOut(int power, int pgn, int cup1, int cup2, int steam){
    uint16_t data = 0;
    data |= (!power << powerLED);
    data |= (!pgn << pgnLED);
    data |= (!cup1 << cup1LED);
    data |= (!cup2 << cup2LED);
    data |= (!steam << steamLED);
    return (PCF.valueOut() & ~ledMask) | (ledMask & data);
  }

  public:
  FrontPanelLeds(PCF8575& pcf) : PCF(pcf) {
    ledMask |= (1 << powerLED);
    ledMask |= (1 << pgnLED);
    ledMask |= (1 << cup1LED);
    ledMask |= (1 << cup2LED);
    ledMask |= (1 << steamLED);
  };

  void setState(FrontLedsState state){
    currentState = state;
  };

  FrontLedsState getState(){
    return currentState;
  };

  void updateLeds(){
    if (currentState != previousState) {
      blinkTimer = millis();
      previousState = currentState;
    }
    switch (currentState) {
      case FrontLedsState::LEDS_OFF:
        dataOut = getDataOut(0, 0, 0, 0, 0);
        break;
      case FrontLedsState::IDDLE:
        dataOut = getDataOut(1, 1, 1, 1, 0);
        break;
      case FrontLedsState::STARTUP:
        if ((millis() - blinkTimer) % 1000 < 500){
          dataOut = getDataOut(1, 1, 1, 1, 0);
        }
        else{
          dataOut = getDataOut(0, 0, 0, 0, 0);
        }
        break;
      case FrontLedsState::BREWING:
        if ((millis() - blinkTimer) % 1000 < 500){
          dataOut = getDataOut(1, 1, 1, 0, 0);
        }
        else{
          dataOut = getDataOut(1, 1, 1, 1, 0);
        }
        break;
      case FrontLedsState::FLUSHING:
        if ((millis() - blinkTimer) % 1000 < 500){
          dataOut = getDataOut(1, 1, 0, 1, 0);
        }
        else{
          dataOut = getDataOut(1, 1, 1, 1, 0);
        }
        break;
      case FrontLedsState::HOTWATER:
        dataOut = getDataOut(1, 1, 1, 1, 1);
        break;
      case FrontLedsState::STEAM_HEATING:
        if ((millis() - blinkTimer) % 1000 < 500){
          dataOut = getDataOut(1, 1, 1, 1, 1);
        }
        else{
          dataOut = getDataOut(1, 1, 1, 1, 0);
        }
        break;
      case FrontLedsState::STEAM_READY:
        dataOut = getDataOut(1, 1, 1, 1, 1);
        break;
      case FrontLedsState::STEAMING:
        dataOut = getDataOut(1, 1, 1, 1, 1);
        break;
      case FrontLedsState::STEAM_FLUSHING:
        if ((millis() - blinkTimer) % 1000 < 500){
          dataOut = getDataOut(1, 1, 0, 1, 1);
        }
        else{
          dataOut = getDataOut(1, 1, 1, 1, 1);
        }
        break;
    }
    if (dataOut != PCF.valueOut()) {
      PCF.write16(dataOut);
    }
  }
};

#endif