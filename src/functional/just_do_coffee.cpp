/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee.h"
#include "../lcd/lcd.h"

unsigned long steamTime;
// inline static float TEMP_DELTA(float d) { return (d*DELTA_RANGE); }
inline static float TEMP_DELTA(float d, const SensorState &currentState) {
  return (
    d * (currentState.pumpFlow < 1.f
      ? currentState.pumpFlow / 9.f
      : currentState.pumpFlow / 10.f
    )
  );
}

void justDoCoffee(const eepromValues_t &runningCfg, const SensorState &currentState) {
  lcdTargetState((int)HEATING::MODE_brew); // setting the target mode to "brew temp"
  float brewTempSetPoint = ACTIVE_PROFILE(runningCfg).setpoint + runningCfg.offsetTemp;
  float sensorTemperature = currentState.temperature + runningCfg.offsetTemp;

  if (currentState.brewActive) { //if brewState == true
    if(sensorTemperature <= brewTempSetPoint - 5.f) {
      setBoilerOn();
    } else {
      float deltaOffset = 0.f;
      if (runningCfg.brewDeltaState) {
        float tempDelta = TEMP_DELTA(brewTempSetPoint, currentState);
        float BREW_TEMP_DELTA = mapRange(sensorTemperature, brewTempSetPoint, brewTempSetPoint + tempDelta, tempDelta, 0, 0);
        deltaOffset = constrain(BREW_TEMP_DELTA, 0, tempDelta);
      }
      if (sensorTemperature <= brewTempSetPoint + deltaOffset) {
        // pulseHeaters(runningCfg.hpwr, (float)runningCfg.mainDivider / 10.f, (float)runningCfg.brewDivider / 10.f, brewActive);
        setHeatersPower(runningCfg.hpwr, (float)runningCfg.brewDivider / 100.f);
      } else {
        setBoilerOff();
      }
    }
  } else if (currentState.flushActive){
    (sensorTemperature <= brewTempSetPoint + 2) ? setBoilerOn() : setBoilerOff();
  } else if (!currentState.steamActive && !currentState.hotWaterActive){ //if brewState == false
    if (sensorTemperature <= ((float)brewTempSetPoint - 35.f)) {
      setHeatersPower(runningCfg.hpwr, 1.f);
    } else if (sensorTemperature <= ((float)brewTempSetPoint - 20.f)) {
      // pulseHeaters(HPWR_OUT, 1.f, (float)runningCfg.mainDivider / 10.f, brewActive);
      setHeatersPower(runningCfg.hpwr, 0.6f);
    } else if (sensorTemperature < ((float)brewTempSetPoint)) {
      // pulseHeaters(HPWR_OUT,  (float)runningCfg.brewDivider / 10.f, (float)runningCfg.brewDivider / 10.f, brewActive);
      setHeatersPower(runningCfg.hpwr, (float)runningCfg.mainDivider / 100.f);
    } else {
      setBoilerOff();
    }
  }

}

void pulseHeaters(const uint32_t pulseLength, const float factor_1, const float factor_2, const bool brewActive) {
  static uint32_t heaterWave;
  static bool heaterState;
  if (!heaterState && ((millis() - heaterWave) > (pulseLength * factor_1))) {
    brewActive ? setBoilerOff() : setBoilerOn();
    heaterState=!heaterState;
    heaterWave=millis();
  } else if (heaterState && ((millis() - heaterWave) > (pulseLength / factor_2))) {
    brewActive ? setBoilerOn() : setBoilerOff();
    heaterState=!heaterState;
    heaterWave=millis();
  }
}

void setHeatersPower(const uint32_t cicleLength, const float powerFactor) {
  static uint32_t heaterWave;
  static bool heaterState;
  const float power = constrain(powerFactor, 0.f, 1.f);
  if (!heaterState && ((millis() - heaterWave) > (cicleLength * power))) {
    setBoilerOff();
    heaterState=!heaterState;
    heaterWave=millis();
  } else if (heaterState && ((millis() - heaterWave) > (cicleLength * (1 - power)))) {
    setBoilerOn();
    heaterState=!heaterState;
    heaterWave=millis();
  }
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState) {
  currentState.steamActive ? lcdTargetState((int)HEATING::MODE_steam) : lcdTargetState((int)HEATING::MODE_brew); // setting the steam/hot water target temp
  // steam temp control, needs to be aggressive to keep steam pressure acceptable
  float steamTempSetPoint = runningCfg.steamSetPoint + runningCfg.offsetTemp;
  float sensorTemperature = currentState.temperature + runningCfg.offsetTemp;
  static bool readyToSteam = false;
  static bool flushingStarted = false;
  static int flushStartTime;
  static double flushingDeltaT = 0;

  if (currentState.steamSwitchState) steamTime = millis();
  if (currentState.temperature > 135.f) readyToSteam = true;

  if (millis() - steamTime >= STEAM_TIMEOUT || flushingStarted){
    readyToSteam = false;
    setBoilerOff();
  } else if (sensorTemperature > steamTempSetPoint + 5.f ) {
    setBoilerOff();
  } else if (sensorTemperature > steamTempSetPoint){
      (readyToSteam && currentState.steamSwitchState) ? setHeatersPower(runningCfg.hpwr, 0.7f) : setBoilerOff();
  } else {
    setBoilerOn();
  }

  if (!flushingStarted && !currentState.steamSwitchState && (currentState.brewSwitchState || currentState.flushSwitchState)){
    flushingStarted = true;
    flushStartTime = millis();
    flushingDeltaT = fmax(0, currentState.temperature - ACTIVE_PROFILE(runningCfg).setpoint);
    currentState.brewSwitchState = false;
    currentState.flushSwitchState = false;

  } else{
    currentState.brewSwitchState = false;
    currentState.flushSwitchState = false;
  }

  if (flushingStarted){
    if (millis() - flushStartTime < (long) (200. * flushingDeltaT)){
      setPumpFullOn();
    }else{
      setPumpOff();
      currentState.steamActive = false;
      flushingStarted = false;
    }
  } else if (readyToSteam && currentState.steamSwitchState){
     setPumpToPercentage(0.05);
  } else {
    setPumpOff();
  }

  /*In case steam is forgotten ON for more than 3 min*/
  if (millis() - steamTime >= STEAM_TIMEOUT && currentState.temperature < ACTIVE_PROFILE(runningCfg).setpoint){
    currentState.steamActive = false;
    readyToSteam = false;
    flushingStarted = false;
    }
}

/*Water mode and all that*/
void hotWaterMode(SensorState &currentState) {
  setPumpToPercentage(1.0);
  setBoilerOn();
  if (currentState.temperature < MAX_WATER_TEMP) setBoilerOn();
  else setBoilerOff();
  currentState.brewSwitchState = false;
  currentState.flushSwitchState = false;
}
