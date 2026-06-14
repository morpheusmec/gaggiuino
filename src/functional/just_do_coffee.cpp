/* 09:32 15/03/2023 - change triggering comment */
#include "just_do_coffee.h"

unsigned long steamTime;
float heatAdded;
uint32_t lastHeatTime;
PIDController controller(
  0.05f, 
  0.f,
  0.f
);

void resetHeating(void){
  controller.reset();
  heatAdded = 0.f;
  lastHeatTime = micros();
}

// inline static float TEMP_DELTA(float d) { return (d*DELTA_RANGE); }
inline static float TEMP_DELTA(float d, const SensorState &currentState) {
  return (
    d * (currentState.pumpFlow < 1.f
      ? currentState.pumpFlow / 9.f
      : currentState.pumpFlow / 10.f
    )
  );
}

void justDoCoffee(const GaggiaSettings &settings, SensorState &currentState,  float waterTemperature) {
  float brewTempSetPoint = waterTemperature + settings.boiler.offsetTemp;
  float sensorTemperature = currentState.temperature + settings.boiler.offsetTemp;
  uint32_t heatTime = micros();
  float elapsedTime = (heatTime - lastHeatTime) / 1000000.f;
  lastHeatTime = heatTime;



  if (currentState.brewActive) {
    heatAdded += getCurrentHeaterLoad() * elapsedTime;
    float pidHeat = controller.calculate(brewTempSetPoint, sensorTemperature, elapsedTime);
    float heatPower = currentState.smoothedPumpFlow * (0.18f + pidHeat);
    heatPower = constrain(heatPower + 0.02f + pidHeat * 2, 0.f, 1.f);
    (sensorTemperature >= brewTempSetPoint + 10.f) ? setBoilerOff() : setHeatersPower(heatPower);
  } else if (currentState.flushActive){
    (sensorTemperature <= brewTempSetPoint + 2) ? setBoilerOn() : setBoilerOff();
  } else if (!currentState.steamActive && !currentState.hotWaterActive){ //if brewState == false
    if (sensorTemperature <= ((float)brewTempSetPoint - 35.f)) {
      setHeatersPower(1.f);
    } else if (sensorTemperature <= ((float)brewTempSetPoint - 20.f)) {
      setHeatersPower((float)settings.boiler.mainDivider / 100.f);  //keeping dividers names at first, although they mean something else
    } else if (sensorTemperature < ((float)brewTempSetPoint) - 0.5f) {
      setHeatersPower((float)settings.boiler.brewDivider / 100.f);
    } else if (sensorTemperature < ((float)brewTempSetPoint)) {
      setHeatersPower(0.04f);
    } else {
      setBoilerOff();
    }
  }

}

void setHeatersPower(const float powerFactor) {
  setHeaterToPercentage(powerFactor);
}

void setBoilerOn(){
  setHeaterToPercentage(1.f);
}

void setBoilerOff(){
  setHeaterToPercentage(0.f);
}

//#############################################################################################
//################################____STEAM_POWER_CONTROL____##################################
//#############################################################################################
void steamCtrl(const GaggiaSettings &settings, SensorState &currentState, float iddleWaterTemperature) {
  // steam temp control, needs to be aggressive to keep steam pressure acceptable
  float steamTempSetPoint = settings.boiler.steamSetPoint + settings.boiler.offsetTemp;
  float sensorTemperature = currentState.temperature + settings.boiler.offsetTemp;
  static bool readyToSteam = false;
  static bool flushingStarted = false;
  static uint32_t flushStartTime;
  static double flushingDeltaT = 0;

  if (currentState.steamSwitchState) steamTime = millis();
  if (currentState.temperature > steamTempSetPoint - 15.f) readyToSteam = true;

  if (millis() - steamTime >= STEAM_TIMEOUT || flushingStarted){
    readyToSteam = false;
    setBoilerOff();
  } else if (sensorTemperature > steamTempSetPoint + 5.f ) {
    setBoilerOff();
  } else if (sensorTemperature > steamTempSetPoint){
      (readyToSteam && currentState.steamSwitchState) ? setHeatersPower(0.85f) : setBoilerOff();
  } else {
    setBoilerOn();
  }

  if (!flushingStarted && !currentState.steamSwitchState && (currentState.brewSwitchState || currentState.flushSwitchState)){
    flushingStarted = true;
    flushStartTime = millis();
    flushingDeltaT = fmax(0, currentState.temperature - iddleWaterTemperature);
    currentState.brewSwitchState = false;
    currentState.flushSwitchState = false;
  } else{
    currentState.brewSwitchState = false;
    currentState.flushSwitchState = false;
  }

  if (flushingStarted){
    frontPanelLeds.setState(FrontLedsState::STEAM_FLUSHING);
    if (millis() - flushStartTime < (uint32_t) (200. * flushingDeltaT)){
      setPumpFullOn();
    }else{
      setPumpOff();
      currentState.steamActive = false;
      flushingStarted = false;
    }
  } else if (!readyToSteam){
    frontPanelLeds.setState(FrontLedsState::STEAM_HEATING);
  } else if (currentState.steamSwitchState){
    frontPanelLeds.setState(FrontLedsState::STEAMING);
    setPumpToPercentage(0.05);
  } else {
    frontPanelLeds.setState(FrontLedsState::STEAM_READY);
    setPumpOff();
  }

  /*In case steam is forgotten ON for more than 3 min*/
  if (millis() - steamTime >= STEAM_TIMEOUT && currentState.temperature < iddleWaterTemperature){
    currentState.steamActive = false;
    readyToSteam = false;
    flushingStarted = false;
  }
}

/*Water mode and all that*/
void hotWaterMode(SensorState &currentState) {
  setPumpToPercentage(0.5);
  setBoilerOn();
  if (currentState.temperature < MAX_WATER_TEMP) setBoilerOn();
  else setBoilerOff();
  currentState.brewSwitchState = false;
  currentState.flushSwitchState = false;
}
