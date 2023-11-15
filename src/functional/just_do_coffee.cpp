/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee.h"
#include "../lcd/lcd.h"

extern unsigned long steamTime;
// inline static float TEMP_DELTA(float d) { return (d*DELTA_RANGE); }
inline static float TEMP_DELTA(float d, const SensorState &currentState) {
  return (
    d * (currentState.pumpFlow < 1.f
      ? currentState.pumpFlow / 9.f
      : currentState.pumpFlow / 10.f
    )
  );
}

void justDoCoffee(const eepromValues_t &runningCfg, const SensorState &currentState, const bool brewActive, const bool flushActive) {
  lcdTargetState((int)HEATING::MODE_brew); // setting the target mode to "brew temp"
  float brewTempSetPoint = ACTIVE_PROFILE(runningCfg).setpoint + runningCfg.offsetTemp;
  float sensorTemperature = currentState.temperature + runningCfg.offsetTemp;

  if (brewActive) { //if brewState == true
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
  } else if (flushActive){
    (sensorTemperature <= brewTempSetPoint + 2) ? setBoilerOn() : setBoilerOff();
  } else { //if brewState == false
    if (sensorTemperature <= ((float)brewTempSetPoint - 20.f)) {
      setBoilerOn();
    } else {
      // int HPWR_LOW =(int)((float)runningCfg.hpwr / ((float)runningCfg.mainDivider / 10.f));
      // Calculating the boiler heating power range based on the below input values
      // int HPWR_OUT = mapRange(sensorTemperature, brewTempSetPoint - 15, brewTempSetPoint, runningCfg.hpwr, HPWR_LOW, 0);
      // HPWR_OUT = constrain(HPWR_OUT, HPWR_LOW, runningCfg.hpwr);  // limits range of sensor values to HPWR_LOW and HPWR

      if (sensorTemperature <= ((float)brewTempSetPoint - 10.f)) {
        // pulseHeaters(HPWR_OUT, 1.f, (float)runningCfg.mainDivider / 10.f, brewActive);
        setHeatersPower(runningCfg.hpwr, (1.f + (float)runningCfg.mainDivider / 100.f) / 2.f);
      } else if (sensorTemperature < ((float)brewTempSetPoint)) {
        // pulseHeaters(HPWR_OUT,  (float)runningCfg.brewDivider / 10.f, (float)runningCfg.brewDivider / 10.f, brewActive);
        setHeatersPower(runningCfg.hpwr, (float)runningCfg.mainDivider / 100.f);
      } else {
        setBoilerOff();
      }
    }
  }
  if (brewActive || !currentState.brewSwitchState) { // keep steam boiler supply valve open while steaming/descale only
    setSteamValveRelayOff();
  }
  setSteamBoilerRelayOff();
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
  if (!heaterState && ((millis() - heaterWave) > (cicleLength * powerFactor))) {
    setBoilerOff();
    heaterState=!heaterState;
    heaterWave=millis();
  } else if (heaterState && ((millis() - heaterWave) > (cicleLength * (1 - powerFactor)))) {
    setBoilerOn();
    heaterState=!heaterState;
    heaterWave=millis();
  }
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const eepromValues_t &runningCfg, SensorState &currentState) {
  currentState.steamSwitchState ? lcdTargetState((int)HEATING::MODE_steam) : lcdTargetState((int)HEATING::MODE_brew); // setting the steam/hot water target temp
  // steam temp control, needs to be aggressive to keep steam pressure acceptable
  float steamTempSetPoint = runningCfg.steamSetPoint + runningCfg.offsetTemp;
  float sensorTemperature = currentState.temperature + runningCfg.offsetTemp;

  if (currentState.smoothedPressure > steamThreshold_ || sensorTemperature > steamTempSetPoint) {
    setBoilerOff();
    setSteamBoilerRelayOff();
    setSteamValveRelayOff();
    setPumpOff();
  } else {
    if (sensorTemperature < steamTempSetPoint) {
      setBoilerOn();
    } else {
      setBoilerOff();
    }
    setSteamValveRelayOn();
    setSteamBoilerRelayOn();
    #ifndef DREAM_STEAM_DISABLED // disabled for bigger boilers which have no  need of adding water during steaming
      if (currentState.smoothedPressure < activeSteamPressure_) {
        setPumpToRawValue(3);
      } else {
        setPumpOff();
      }
    #endif
  }

  /*In case steam is forgotten ON for more than 15 min*/
  if (currentState.smoothedPressure > passiveSteamPressure_) {
    currentState.isSteamForgottenON = millis() - steamTime >= STEAM_TIMEOUT;
  } else steamTime = millis();
}

/*Water mode and all that*/
void hotWaterMode(const SensorState &currentState) {
  closeValve();
  setPumpToRawValue(80);
  setBoilerOn();
  if (currentState.temperature < MAX_WATER_TEMP) setBoilerOn();
  else setBoilerOff();
}
