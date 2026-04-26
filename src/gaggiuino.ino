/* 09:32 15/03/2023 - change triggering comment */
#pragma GCC optimize ("Ofast")
#define STM32F4 // This define has to be here otherwise the include of FlashStorage_STM32.h bellow fails.
#include <FlashStorage_STM32.h>
#if defined(DEBUG_ENABLED)
  #include "dbg.h"
#endif
#include "gaggiuino.h"

SimpleKalmanFilter smoothPressure(0.15f, 0.15f, 0.2f);
SimpleKalmanFilter smoothPumpFlow(0.1f, 0.1f, 0.01f);
SimpleKalmanFilter smoothScalesFlow(0.5f, 0.5f, 0.01f);
SimpleKalmanFilter smoothConsideredFlow(0.1f, 0.1f, 0.1f);

//default phases. Updated in updateProfilerPhases.
Profile profile;
PhaseProfiler phaseProfiler{profile};

PredictiveWeight predictiveWeight;

SensorState currentState;

OPERATION_MODES selectedOperationalMode;

GaggiaSettings runningCfg;

SystemState systemState;

void setup(void) {
  LOG_INIT();
  delay(1000);
  LOG_INFO("Gaggiuino (fw: %s) booting", AUTO_VERSION);

  // Various pins operation mode handling
  pinInit();
  LOG_INFO("Pin init");
  
  lcdInit();
  LOG_INFO("LCD Init");

  #if defined(DEBUG_ENABLED)
    // Debug init if enabled
    dbgInit();
    LOG_INFO("DBG init");
  #endif

  // Init the tof sensor
  currentState.waterLvl = 30u;

  // Initialise comms library for talking to the ESP mcu
  espCommsInit();

  // Initialising the saved values or writing defaults if first start
  eepromInit();
  runningCfg = eepromGetCurrentSettings();
  LOG_INFO("EEPROM Init");

  cpsInit(runningCfg);
  LOG_INFO("CPS Init");

  thermocoupleInit();
  LOG_INFO("Thermocouple Init");

  lcdUploadCfg(runningCfg);
  LOG_INFO("LCD cfg uploaded");

  adsInit();
  LOG_INFO("Pressure sensor init");

  // Scales handling
  scalesInit(runningCfg.system.scalesF1);
  LOG_INFO("Scales init");

  pageValuesRefresh();
  LOG_INFO("Setup sequence finished");

  iwdcInit();
}

//##############################################################################################################################
//############################################________________MAIN______________################################################
//##############################################################################################################################


//Main loop where all the logic is continuously run
void loop(void) {
  if (lcdCurrentPageId != lcdLastCurrentPageId) pageValuesRefresh();
  lcdListen();
  sensorsRead();
  modeDetect();
  relaysActuate();
  modeSelect();
  lcdRefresh();
  espCommsSendSensorData(currentState, 100);
  sysHealthCheck();
}

//##############################################################################################################################
//#############################################___________SENSORS_READ________##################################################
//##############################################################################################################################


static void sensorsRead(void) {
  sensorReadSwitches();
  espCommsReadData();
  sensorsReadTemperature();
  sensorsReadWeight();
  sensorsReadPressure();
  calculateWeightAndFlow();
  updateStartupTimer();
}

static bool cup1_read_switch(void){
  static unsigned long last_time = 0;
  static bool last_reading = false;
  static bool last_state = false;
  bool press = false;
  bool reading = cup1BtnState();
  
  if (reading != last_reading) last_time = millis();
  last_reading = reading;
  if ((millis() - last_time) >= 1){
    if(last_state != reading){
      if (reading) press = true;
    last_state = reading;
    }
  }
  return press;
}

static bool cup2_read_switch(void){
  static unsigned long last_time = 0;
  static bool last_reading = false;
  static bool last_state = false;
  bool press = false;
  bool reading = cup2BtnState();
  
  if (reading != last_reading) last_time = millis();
  last_reading = reading;
  if ((millis() - last_time) >= 1){
    if(last_state != reading){
      if (reading) press = true;
    last_state = reading;
    }
  }
  return press;
}

static void sensorReadSwitches(void) {
  bool cup1_press = cup1_read_switch();
  bool cup2_press = cup2_read_switch();

  if (cup1_press || cup2_press){
    if (currentState.brewSwitchState || currentState.flushSwitchState) {
      currentState.brewSwitchState = false;
      currentState.flushSwitchState = false;
    } 
    else{
      if (cup1_press){
        currentState.flushSwitchState = true;
        brewParamsReset();
      }
      else{
        currentState.brewSwitchState = true;
      }
    } 
  }

  currentState.steamSwitchState = steamBtnState();
  currentState.hotWaterSwitchState = waterBtnState();
}

static void sensorsReadTemperature(void) {
  if (tempReady()) {
    currentState.temperature = thermocoupleRead() - runningCfg.boiler.offsetTemp;
  }
}

static void relaysActuate(void) {
  if (selectedOperationalMode == OPERATION_MODES::OPMODE_flush || selectedOperationalMode == OPERATION_MODES::OPMODE_descale) return;
  static bool relWasNeeded = (currentState.brewActive || currentState.flushActive);
  static unsigned int shotEndTimer = -100000;
  bool relNeeded = (currentState.brewActive || currentState.flushActive);

  if (relNeeded && !relWasNeeded){
    setSol2Off();
    setSol3On();
    delay(100);
  }
  else if (!relNeeded && relWasNeeded){
    shotEndTimer = millis();
    setPumpOff();
  }

  if (!relNeeded){
    ((millis() - shotEndTimer > 60) && (millis() - shotEndTimer < 10000)) ? setSol2On() : setSol2Off();
    if (millis() - shotEndTimer > 150) setSol3Off();
  }
  
  relWasNeeded = relNeeded;
}

static void sensorsReadWeight(void) {
  uint32_t elapsedTime = millis() - scalesTimer;

  if (elapsedTime > GET_SCALES_READ_EVERY) {
    currentState.scalesPresent = scalesIsPresent();
    if (currentState.scalesPresent) {
      if (currentState.tarePending) {
        scalesTare();
        weightMeasurements.clear();
        weightMeasurements.add(scalesGetWeight());
        currentState.tarePending = false;
      }
      else {
        weightMeasurements.add(scalesGetWeight());
      }
      currentState.weight = abs(weightMeasurements.latest().value) > 0.3f ? weightMeasurements.latest().value : 0.f;

      if (currentState.brewActive) {
        currentState.shotWeight = currentState.tarePending ? 0.f : currentState.weight;
        currentState.weightFlow = weightMeasurements.measurementChange().changeSpeed();
        currentState.smoothedWeightFlow = smoothScalesFlow.updateEstimate(currentState.weightFlow);
      }
    }
    scalesTimer = millis();
  }
}

static void sensorsReadPressure(void) {
  uint32_t elapsedTime = millis() - pressureTimer;

  if (elapsedTime > GET_PRESSURE_READ_EVERY) {
    float elapsedTimeSec = elapsedTime / 1000.f;
    currentState.pressure = max(0.f, getPressure());
    previousSmoothedPressure = currentState.smoothedPressure;
    currentState.smoothedPressure = smoothPressure.updateEstimate(currentState.pressure);
    currentState.pressureChangeSpeed = (currentState.smoothedPressure - previousSmoothedPressure) / elapsedTimeSec;
    pressureTimer = millis();
  }
}

void sensorsReadFlow(float elapsedTimeSec) {
  currentState.pumpLoad = getAndResetLoadAverage();
  currentState.pumpFlow = findQ(currentState.smoothedPressure, currentState.pumpLoad);
  previousSmoothedPumpFlow = currentState.smoothedPumpFlow;
  // Some flow smoothing
  currentState.smoothedPumpFlow = smoothPumpFlow.updateEstimate(currentState.pumpFlow);
  currentState.pumpFlowChangeSpeed = (currentState.smoothedPumpFlow - previousSmoothedPumpFlow) / elapsedTimeSec;
}

static void calculateWeightAndFlow(void) {
  uint32_t elapsedTime = millis() - flowTimer;

  if (currentState.brewActive) {
    // Marking for tare in case smth has gone wrong and it has exited tare already.
    if (currentState.weight < -.3f) currentState.tarePending = true;

    if (elapsedTime > REFRESH_FLOW_EVERY) {
      flowTimer = millis();
      float elapsedTimeSec = elapsedTime / 1000.f;
      sensorsReadFlow(elapsedTimeSec);
      float consideredFlow = currentState.smoothedPumpFlow * elapsedTimeSec;
      // Update predictive class with our current phase
      const CurrentPhase& phase = phaseProfiler.getCurrentPhase();
      predictiveWeight.update(currentState, phase, runningCfg);

      // Start the predictive weight calculations when conditions are true
      if (predictiveWeight.isOutputFlow() || currentState.weight > 0.4f) {
        float flowrate = findQ(currentState.smoothedPressure, getCurrentPumpLoad());
        float actualFlow = flowrate * elapsedTimeSec;
        /* Probabilistically the flow is lower if the shot is just started winding up and we're flow profiling,
        once pressure stabilises around the setpoint the flow is either stable or puck restriction is high af. */
        // if ((ACTIVE_PROFILE(runningCfg).mfProfileState || ACTIVE_PROFILE(runningCfg).tpType) && currentState.pressureChangeSpeed > 0.15f) {
        //   if ((currentState.smoothedPressure < ACTIVE_PROFILE(runningCfg).mfProfileStart * 0.9f)
        //   || (currentState.smoothedPressure < ACTIVE_PROFILE(runningCfg).tfProfileStart * 0.9f)) {
        //     actualFlow *= 0.3f;
        //   }
        // }
        currentState.consideredFlow = smoothConsideredFlow.updateEstimate(actualFlow);
        currentState.shotWeight = currentState.scalesPresent ? currentState.shotWeight : currentState.shotWeight + actualFlow;
      }
      currentState.waterPumped += consideredFlow;
    }
  } else {
    currentState.consideredFlow = 0.f;
    currentState.pumpCPS = getAndResetClickCounter();
    getAndResetLoadAverage();
    flowTimer = millis();
  }
}

//##############################################################################################################################
//############################################______PAGE_CHANGE_VALUES_REFRESH_____#############################################
//##############################################################################################################################
static void pageValuesRefresh() {
  // Read the page we're landing in: leaving keyboard page means a value could've changed in it
  if (lcdLastCurrentPageId == NextionPage::KeyboardNumeric) lcdFetchPage(runningCfg, lcdCurrentPageId, runningCfg.profiles.activeProfileIndex);
  // Or maybe it's a page that needs constant polling
  else if (lcdLastCurrentPageId == NextionPage::Led) lcdFetchPage(runningCfg, lcdCurrentPageId, runningCfg.profiles.activeProfileIndex);
  // Finally read the page we left, as it could've been changed in place (e.g. boolean toggles)
  else lcdFetchPage(runningCfg, lcdLastCurrentPageId, runningCfg.profiles.activeProfileIndex);

  homeScreenScalesEnabled = lcdGetHomeScreenScalesEnabled();
  // MODE_SELECT should always be LAST
  selectedOperationalMode = (OPERATION_MODES) lcdGetSelectedOperationalMode();

  lcdLastCurrentPageId = lcdCurrentPageId;
}

//#############################################################################################
//############################____OPERATIONAL_MODE_CONTROL____#################################
//#############################################################################################
static void modeSelect(void) {

  switch (selectedOperationalMode) {
    //REPLACE ALL THE BELOW WITH OPMODE_auto_profiling
    case OPERATION_MODES::OPMODE_straight9Bar:
    case OPERATION_MODES::OPMODE_justPreinfusion:
    case OPERATION_MODES::OPMODE_justPressureProfile:
    case OPERATION_MODES::OPMODE_preinfusionAndPressureProfile:
    case OPERATION_MODES::OPMODE_flowPreinfusionStraight9BarProfiling:
    case OPERATION_MODES::OPMODE_justFlowBasedProfiling:
    case OPERATION_MODES::OPMODE_FlowBasedPreinfusionPressureBasedProfiling:
    case OPERATION_MODES::OPMODE_everythingFlowProfiled:
    case OPERATION_MODES::OPMODE_pressureBasedPreinfusionAndFlowProfile:
      if (currentState.hotWaterActive) hotWaterMode(currentState);
      else if (currentState.steamActive) steamCtrl(runningCfg, currentState);
      else {
        profiling();
      }
      break;
    case OPERATION_MODES::OPMODE_manual:
      manualFlowControl();
      break;
    case OPERATION_MODES::OPMODE_flush:
      backFlush(currentState);
      justDoCoffee(runningCfg, currentState);
      break;
    case OPERATION_MODES::OPMODE_steam:
      steamCtrl(runningCfg, currentState);
      break;
    case OPERATION_MODES::OPMODE_descale:
      deScale(runningCfg, currentState);
      break;
    default:
      pageValuesRefresh();
      break;
  }
}

//#############################################################################################
//################################____LCD_REFRESH_CONTROL___###################################
//#############################################################################################

static void lcdRefresh(void) {
  uint16_t tempDecimal;

  if (millis() > pageRefreshTimer) {
    /*LCD pressure output, as a measure to beautify the graphs locking the live pressure read for the LCD alone*/
    #ifdef BEAUTIFY_GRAPH
      lcdSetPressure(currentState.smoothedPressure * 10.f);
    #else
      lcdSetPressure(
        currentState.pressure > 0.f
          ? currentState.pressure * 10.f
          : 0.f
      );
    #endif

    /*LCD temp output*/
    currentState.waterTemperature = currentState.temperature;

    lcdSetTemperature(std::floor((uint16_t)currentState.waterTemperature));

    /*LCD weight & temp & water lvl output*/
    switch (lcdCurrentPageId) {
      case NextionPage::Home:
        // temp decimal handling
        tempDecimal = (currentState.waterTemperature - (uint16_t)currentState.waterTemperature) * 10;
        lcdSetTemperatureDecimal(tempDecimal);
        // water lvl
        lcdSetTankWaterLvl(currentState.waterLvl);
        //weight
        if (homeScreenScalesEnabled) lcdSetWeight(currentState.weight);
        break;
      case NextionPage::BrewGraph:
      case NextionPage::BrewManual:
        // temp decimal handling
        tempDecimal = (currentState.waterTemperature - (uint16_t)currentState.waterTemperature) * 10;
        lcdSetTemperatureDecimal(tempDecimal);
        // If the weight output is a negative value lower than -0.8 you might want to tare again before extraction starts.
        if (currentState.shotWeight) lcdSetWeight(currentState.shotWeight > -0.8f ? currentState.shotWeight : -0.9f);
        /*LCD flow output*/
        lcdSetFlow( currentState.smoothedPumpFlow * 10.f);
        break;
      default:
        break; // don't push needless data on other pages
    }

  #ifdef DEBUG_ENABLED
    lcdShowDebug(readTempSensor(), getAdsError());
  #endif

    /*LCD timer and warmup*/
    if (currentState.brewActive) {
      lcdSetBrewTimer((millis() > brewingTimer) ? (int)((millis() - brewingTimer) / 1000) : 0);
      lcdBrewTimerStart(); // nextion timer start
      lcdWarmupStateStop(); // Flagging warmup notification on Nextion needs to stop (if enabled)
    } else {
      lcdBrewTimerStop(); // nextion timer stop
    }

    pageRefreshTimer = millis() + REFRESH_SCREEN_EVERY;
  }
}
//#############################################################################################
//###################################____SAVE_BUTTON____#######################################
//#############################################################################################
void tryEepromWrite(const GaggiaSettings& settings) {
  bool success = eepromWrite(settings);
  watchdogReload(); // reload the watchdog timer on expensive operations
  if (success) {
    lcdShowPopup("Update successful!");
  } else {
    lcdShowPopup("Data out of range!");
  }
}

void lcdSwitchActiveToStoredProfile(const GaggiaSettings& storedSettings) {
  runningCfg.profiles.activeProfileIndex = lcdGetSelectedProfile();
  ACTIVE_PROFILE(runningCfg) = storedSettings.profiles.savedProfiles[runningCfg.profiles.activeProfileIndex];
  lcdUploadProfile(runningCfg);
}

// Save the desired temp values to EEPROM
void lcdSaveSettingsTrigger(void) {
  LOG_VERBOSE("Saving values to EEPROM");
  GaggiaSettings eepromCurrentValues = eepromGetCurrentSettings();
  lcdFetchPage(eepromCurrentValues, lcdCurrentPageId, runningCfg.profiles.activeProfileIndex);
  tryEepromWrite(eepromCurrentValues);
}

void lcdSaveProfileTrigger(void) {
  LOG_VERBOSE("Saving profile to EEPROM");

  GaggiaSettings currentSettings = eepromGetCurrentSettings();
  lcdFetchCurrentProfile(currentSettings);
  tryEepromWrite(currentSettings);
}

void lcdResetSettingsTrigger(void) {
  tryEepromWrite(eepromGetDefaultSettings());
}

void lcdLoadDefaultProfileTrigger(void) {
  lcdSwitchActiveToStoredProfile(eepromGetDefaultSettings());

  lcdShowPopup("Profile loaded!");
}

void onTareReceived() {
  LOG_VERBOSE("Tare scales");
  if (currentState.scalesPresent) currentState.tarePending = true;
}

void lcdScalesTareTrigger(void) {
  onTareReceived();
}

void lcdHomeScreenScalesTrigger(void) {
  LOG_VERBOSE("Scales enabled or disabled");
  homeScreenScalesEnabled = lcdGetHomeScreenScalesEnabled();
}

void lcdBrewGraphScalesTareTrigger(void) {
  LOG_VERBOSE("Predictive scales tare action completed!");
  if (currentState.scalesPresent) {
    currentState.tarePending = true;
  }
  else {
    currentState.shotWeight = 0.f;
    predictiveWeight.setIsForceStarted(true);
  }
}

void lcdRefreshElementsTrigger(void) {
  GaggiaSettings eepromCurrentSettings = eepromGetCurrentSettings();

  // Make the necessary changes
  uploadPageCfg(eepromCurrentSettings, systemState);
  // refresh the screen elements
  pageValuesRefresh();
}

void lcdQuickProfileSwitch(void) {
  lcdSwitchActiveToStoredProfile(eepromGetCurrentSettings());
  lcdShowPopup("Profile switched!");
}

//#############################################################################################
//###############################____PROFILING_CONTROL____#####################################
//#############################################################################################

void onProfileReceived(Profile& newProfile) {
}

static void profiling(void) {
  if (currentState.brewActive) { //runs this only when brew button activated and pressure profile selected
    uint32_t timeInShot = millis() - brewingTimer;
    phaseProfiler.setProfile(ACTIVE_PROFILE(runningCfg));
    phaseProfiler.updatePhase(timeInShot, currentState);
    const CurrentPhase& currentPhase = phaseProfiler.getCurrentPhase();
    ShotSnapshot shotSnapshot = buildShotSnapshot(timeInShot, currentState, phaseProfiler);
    espCommsSendShotData(shotSnapshot, 100);

    if (phaseProfiler.isFinished()) {
      setPumpOff();
      currentState.brewSwitchState = false;
    } else if (currentPhase.getType() == PhaseType::PRESSURE) {
      float newBarValue = currentPhase.getTarget();
      float flowRestriction =  currentPhase.getRestriction();
      setPumpPressure(newBarValue, flowRestriction, currentState);
    } else {
      float newFlowValue = currentPhase.getTarget();
      float pressureRestriction =  currentPhase.getRestriction();
      setPumpFlow(newFlowValue, pressureRestriction, currentState);
    }
  } else if (currentState.flushActive){
    setPumpToPercentage(0.5);
  }
  else {
    setPumpOff();
  }
  // Keep that water at temp
  if (millis() - iddleTimer < 1000 * 60 * 90){ //safety if machine is forgotten on
    justDoCoffee(runningCfg, currentState);
  } else {
    setBoilerOff();
    digitalWrite(shutdownPin, LOW);
  }
}

static void manualFlowControl(void) {
  if (currentState.brewActive) {
    float flow_reading = lcdGetManualFlowVol() / 10.f ;
    setPumpFlow(flow_reading, 0.f, currentState);
  } else {
    setPumpOff();
  }
  justDoCoffee(runningCfg, currentState);
}

//#############################################################################################
//###################################____BREW DETECT____#######################################
//#############################################################################################

static void modeDetect(void) {
  currentState.brewActive = false;
  currentState.flushActive = false;
  currentState.hotWaterActive = false;

  if (selectedOperationalMode == OPERATION_MODES::OPMODE_flush || selectedOperationalMode == OPERATION_MODES::OPMODE_descale){
    return;
  }

  if (currentState.steamSwitchState){
    currentState.steamActive = true;
  } else if (currentState.hotWaterSwitchState){
    currentState.hotWaterActive = true;
    currentState.steamActive = false;
  } else if (!currentState.steamActive && !currentState.hotWaterActive){
    if (currentState.brewSwitchState){
      currentState.brewActive = true;
    } else if (currentState.flushSwitchState){
      currentState.flushActive = true;
    } 
  }

  if (currentState.brewActive || currentState.flushActive || currentState.steamActive || currentState.hotWaterActive){
    iddleTimer = millis();
    lcdWakeUp();
  }

  static bool paramsReset = true;
  if (currentState.brewActive) {
    if (!paramsReset) {
      brewParamsReset();
      paramsReset = true;
    }
    // needs to be here as it creates a locking state soemtimes if not kept up to date during brew
    // mainly when shotWeight restriction kick in.
    systemHealthTimer = millis() + HEALTHCHECK_EVERY;
  } else {
    currentState.pumpCPS = getAndResetClickCounter();
    getAndResetLoadAverage();
    if (paramsReset) {
      brewParamsReset();
      paramsReset = false;
    }
  }
}

static void brewParamsReset(void) {
  currentState.tarePending = true;
  currentState.shotWeight  = 0.f;
  currentState.pumpFlow    = 0.f;
  currentState.weight      = 0.f;
  currentState.waterPumped = 0.f;
  brewingTimer             = millis();
  flowTimer                = brewingTimer;
  systemHealthTimer        = brewingTimer + HEALTHCHECK_EVERY;
  resetHeating();

  weightMeasurements.clear();
  predictiveWeight.reset();
  smoothPumpFlow.resetEstimate();
  smoothScalesFlow.resetEstimate();
  smoothConsideredFlow.resetEstimate();
  phaseProfiler.reset();
}

// Function to track time since system has started
static unsigned long getTimeSinceInit(void) {
  static unsigned long startTime = millis();
  return millis() - startTime;
}

static inline void sysHealthCheck() {
  //Reloading the watchdog timer, if this function fails to run MCU is rebooted
  watchdogReload();

  /* This *while* is here to prevent situations where the system failed to get a temp reading and temp reads as 0 or -7(cause of the offset)
  If we would use a non blocking function then the system would keep the SSR in HIGH mode which would most definitely cause boiler overheating */
  while (currentState.temperature + runningCfg.boiler.offsetTemp <= 0.0f || currentState.temperature == NAN || currentState.temperature + runningCfg.boiler.offsetTemp >= 200.0f) {
    //Reloading the watchdog timer, if this function fails to run MCU is rebooted
    watchdogReload();
    /* In the event of the temp failing to read while the SSR is HIGH
    we force set it to LOW while trying to get a temp reading - IMPORTANT safety feature */
    setPumpOff();
    setBoilerOff();
    if (millis() > thermoTimer) {
      LOG_ERROR("Cannot read temp from thermocouple (last read: %.1lf)!", static_cast<double>(currentState.temperature));
      currentState.steamActive ? lcdShowPopup("COOLDOWN") : lcdShowPopup("TEMP READ ERROR"); // writing a LCD message
      currentState.temperature  = thermocoupleRead() - runningCfg.boiler.offsetTemp;  // Making sure we're getting a value
      thermoTimer = millis() + GET_KTYPE_READ_EVERY;
    }
  }
}

static void updateStartupTimer(void) {
  lcdSetUpTime(getTimeSinceInit() / 1000);
}

static void cpsInit(GaggiaSettings &runningCfg) {
  int cps = getCPS();
  if (cps > 110) { // double 60 Hz
    runningCfg.system.powerLineFrequency = 60u;
  } else if (cps > 80) { // double 50 Hz
    runningCfg.system.powerLineFrequency = 50u;
  } else if (cps > 55) { // 60 Hz
    runningCfg.system.powerLineFrequency = 60u;
  } else if (cps > 0) { // 50 Hz
    runningCfg.system.powerLineFrequency = 50u;
  }
}

