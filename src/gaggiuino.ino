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
bool gaggiaSettingsInitialized = false;
bool activeProfileInitialized = false;

ProfileSettings profileSettings;
Profile manualProfile;
Profile activeProfile;

PhaseProfiler phaseProfiler{activeProfile};

PredictiveWeight predictiveWeight;

SensorState currentState;

PCF8575 PCF(0x20);

OPERATION_MODES selectedOperationalMode;

GaggiaSettings runningCfg;

SystemState systemState;

void setup(void) {
  LOG_INIT();
  // delay(1000);
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
  currentState.waterLevel = 30u;

  // Initialize comms library for talking to the ESP mcu
  espCommsInit();

  // Initializing the saved values or writing defaults if first start
  eepromInit();
  runningCfg = eepromGetCurrentSettings();
  profileSettings = eepromGetCurrentProfiles();
  activeProfile = profileSettings.savedProfiles[profileSettings.activeProfileIndex];
  LOG_INFO("EEPROM Init");

  cpsInit(runningCfg);
  LOG_INFO("CPS Init");

  thermocoupleInit();
  LOG_INFO("Thermocouple Init");

  lcdUploadCfg(runningCfg, profileSettings);
  LOG_INFO("LCD cfg uploaded");

  adsInit();
  LOG_INFO("Pressure sensor init");

  // Scales handling
  scalesInit(runningCfg.scales);
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
  espUpdateState();
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
  if ((millis() - last_time) >= 20){
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
  if ((millis() - last_time) >= 20){
    if(last_state != reading){
      if (reading) press = true;
    last_state = reading;
    }
  }
  return press;
}

static void sensorReadSwitches(void) {
  readPCF();
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
  if (systemState.operationMode == OperationMode::FLUSH || systemState.operationMode == OperationMode::DESCALE) return;
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

static Measurement handleTaringAndReadWeight() {
  if (!systemState.tarePending) { // No tare needed just get weight
    return scalesGetWeight();
  }

  // Tare is required. Invoke it.
  scalesTare();
  weightMeasurements.clear();
  Measurement weight = scalesGetWeight();

  if (fabsf(weight.value) < 0.5f) { // Tare was successful. return reading
    systemState.tarePending = false;
    return weight;
  } else {  // Tare was unsuccessful. return 0 weight.
    return Measurement{ .value=0.f, .millis = millis()};
  }
}

static void sensorsReadWeight(void) {
  uint32_t currentMillis = millis();
  uint32_t elapsedTime = currentMillis - scalesTimer;
  uint32_t weightBumpTimeout = currentMillis - scalesTimeout;
  static float initialWeight = 0.f;

  if (elapsedTime > GET_SCALES_READ_EVERY) {
    currentState.scalesPresent = scalesIsPresent();
    if (currentState.scalesPresent) {
      const auto weight = handleTaringAndReadWeight();
      weightMeasurements.add(weight);
      currentState.weight = abs(weightMeasurements.getLatest().value) > 0.3f ? weightMeasurements.getLatest().value : 0.f;
      if (currentState.brewActive) {
        const float weightFlow = weightMeasurements.getMeasurementChange().speed();
            // If there's a sudden jump in weight
        bool isChangeRateHigh = weightFlow > weightRateThreshold;
        bool isCupPlaced = currentState.weight - initialWeight > 0.f
                        && currentState.weight - initialWeight >= weightIncreaseThreshold;
        if (!systemState.tarePending && (isChangeRateHigh || isCupPlaced)) {
          // Ignore accidental weight bumps
          if (weightBumpTimeout < GET_SCALES_ACCIDENTAL) {
            scalesTimer = currentMillis;
            return;
          } else { // Weight increased drastically and is constant ? mark for tare.
            systemState.tarePending = true;
            scalesTimer = currentMillis;
            scalesTimeout = currentMillis;
          }
        }      
        currentState.shotWeight = systemState.tarePending ? 0.f : currentState.weight;
        initialWeight = currentState.shotWeight;

        // Only take flow measurements when tare is not pending.
        currentState.weightFlow = systemState.tarePending
                                ? currentState.weightFlow
                                : fmax(0.f, weightFlow);
        currentState.smoothedWeightFlow = smoothScalesFlow.updateEstimate(currentState.weightFlow);
      }
    }
    scalesTimer = millis();
    scalesTimeout = currentMillis;
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
    if (currentState.weight < -.3f) systemState.tarePending = true;

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
  if (lcdLastCurrentPageId == NextionPage::KeyboardNumeric) lcdFetchPage(runningCfg, profileSettings, lcdCurrentPageId);
  // Or maybe it's a page that needs constant polling
  else if (lcdLastCurrentPageId == NextionPage::Led) lcdFetchPage(runningCfg, profileSettings, lcdCurrentPageId);
  // Finally read the page we left, as it could've been changed in place (e.g. boolean toggles)
  else lcdFetchPage(runningCfg, profileSettings, lcdLastCurrentPageId);

  homeScreenScalesEnabled = lcdGetHomeScreenScalesEnabled();
  // MODE_SELECT should always be LAST
  selectedOperationalMode = (OPERATION_MODES) lcdGetSelectedOperationalMode();
  switch (selectedOperationalMode) {
    //TODO: temporary 
    case OPERATION_MODES::OPMODE_straight9Bar:
    case OPERATION_MODES::OPMODE_justPreinfusion:
    case OPERATION_MODES::OPMODE_justPressureProfile:
    case OPERATION_MODES::OPMODE_preinfusionAndPressureProfile:
    case OPERATION_MODES::OPMODE_flowPreinfusionStraight9BarProfiling:
    case OPERATION_MODES::OPMODE_justFlowBasedProfiling:
    case OPERATION_MODES::OPMODE_FlowBasedPreinfusionPressureBasedProfiling:
    case OPERATION_MODES::OPMODE_everythingFlowProfiled:
    case OPERATION_MODES::OPMODE_pressureBasedPreinfusionAndFlowProfile:
      systemState.operationMode = OperationMode::BREW_AUTO;
      break;
    case OPERATION_MODES::OPMODE_manual:
      systemState.operationMode = OperationMode::BREW_MANUAL;
      break;
    case OPERATION_MODES::OPMODE_flush:
      systemState.operationMode = OperationMode::FLUSH;
      break;
    case OPERATION_MODES::OPMODE_steam:
      systemState.operationMode = OperationMode::STEAM;
      break;
    case OPERATION_MODES::OPMODE_descale:
      systemState.operationMode = OperationMode::DESCALE;
      break;
    default:
      break;
  }
  lcdLastCurrentPageId = lcdCurrentPageId;
}

//#############################################################################################
//############################____OPERATIONAL_MODE_CONTROL____#################################
//#############################################################################################
static void modeSelect(void) {

  switch (systemState.operationMode) {
    case OperationMode::BREW_AUTO:
      if (currentState.hotWaterActive) hotWaterMode(currentState);
      else if (currentState.steamActive) steamCtrl(runningCfg, currentState, activeProfile.waterTemperature);
      else {
        profiling();
      }
      break;
    case OperationMode::BREW_MANUAL:
      manualFlowControl(); //TODO: change to profiling after disabling nextion
      break;
    case OperationMode::FLUSH:
      backFlush(currentState);
      justDoCoffee(runningCfg, currentState, activeProfile.waterTemperature);
      break;
    case OperationMode::STEAM:
      steamCtrl(runningCfg, currentState, activeProfile.waterTemperature);
      break;
    case OperationMode::DESCALE:
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

  if (millis() > NextionPageRefreshTimer) {
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
        lcdSetTankWaterLvl(currentState.waterLevel);
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

    NextionPageRefreshTimer = millis() + REFRESH_SCREEN_EVERY;
  }
}
//#############################################################################################
//###################################____SAVE_BUTTON____#######################################
//#############################################################################################
void tryEepromWrite(const GaggiaSettings& settings, const ProfileSettings& profileSettings) {
  bool success = eepromWrite(settings, profileSettings);
  watchdogReload(); // reload the watchdog timer on expensive operations
  if (success) {
    lcdShowPopup("Update successful!");
  } else {
    lcdShowPopup("Data out of range!");
  }
}

void lcdSwitchActiveToStoredProfile(ProfileSettings& storedProfiles) {
  storedProfiles.activeProfileIndex = lcdGetSelectedProfile();
  activeProfile = storedProfiles.savedProfiles[storedProfiles.activeProfileIndex];
  lcdUploadProfile(activeProfile, storedProfiles.activeProfileIndex);
}

// Save the desired temp values to EEPROM
void lcdSaveSettingsTrigger(void) {
  LOG_VERBOSE("Saving values to EEPROM");
  GaggiaSettings eepromCurrentValues = eepromGetCurrentSettings();
  lcdFetchPage(eepromCurrentValues, profileSettings, lcdCurrentPageId);
  activeProfile =profileSettings.savedProfiles[profileSettings.activeProfileIndex];
  tryEepromWrite(eepromCurrentValues, profileSettings);
}

void lcdSaveProfileTrigger(void) {
  LOG_VERBOSE("Saving profile to EEPROM");

  GaggiaSettings currentSettings = eepromGetCurrentSettings();
  lcdFetchCurrentProfile(currentSettings, profileSettings);
  activeProfile = profileSettings.savedProfiles[profileSettings.activeProfileIndex];
  tryEepromWrite(currentSettings, profileSettings);
}

void lcdResetSettingsTrigger(void) {
  tryEepromWrite(eepromGetDefaultSettings(), eepromGetDefaultProfiles());
}

void lcdLoadDefaultProfileTrigger(void) {
  ProfileSettings defaultProfiles = eepromGetDefaultProfiles();
  lcdSwitchActiveToStoredProfile(defaultProfiles);

  lcdShowPopup("Profile loaded!");
}

void lcdScalesTareTrigger(void) {
  onTareCommandReceived();
}

void lcdHomeScreenScalesTrigger(void) {
  LOG_VERBOSE("Scales enabled or disabled");
  homeScreenScalesEnabled = lcdGetHomeScreenScalesEnabled();
}

void lcdBrewGraphScalesTareTrigger(void) {
  LOG_VERBOSE("Predictive scales tare action completed!");
  if (currentState.scalesPresent) {
    systemState.tarePending = true;
  }
  else {
    currentState.shotWeight = 0.f;
    predictiveWeight.setIsForceStarted(true);
  }
}

void lcdRefreshElementsTrigger(void) {
  // GaggiaSettings eepromCurrentSettings = eepromGetCurrentSettings();
  // Make the necessary changes
  uploadPageCfg(runningCfg, profileSettings, activeProfile, systemState);
  // refresh the screen elements
  pageValuesRefresh();
}

void lcdQuickProfileSwitch(void) {
  lcdSwitchActiveToStoredProfile(profileSettings);
  lcdShowPopup("Profile switched!");
}

//#############################################################################################
//################################____EPS_COMMS_CONTROL___###################################
//#############################################################################################

static void espUpdateState(void) {
  if (millis() > pageRefreshTimer) {
 
    if (currentState.brewActive && systemState.operationMode == OperationMode::BREW_AUTO) {
      espCommsSendShotData(buildShotSnapshot(millis() - brewingTimer, currentState, phaseProfiler), 100);
    } else {
      espCommsSendSystemState(systemState, 1000);
      espCommsSendSensorData(runningCfg, currentState, activeProfile, 500);
    }
    pageRefreshTimer = millis() + REFRESH_ESP_DATA_EVERY;
  }
}

void onProfileReceived(const Profile& newProfile) {
  activeProfile = newProfile;
  activeProfileInitialized = true;
}

void onGaggiaSettingsReceived(const GaggiaSettings& newSettings) {
  GaggiaSettings previous = runningCfg;
  runningCfg = newSettings;
  if (!gaggiaSettingsInitialized) {
    gaggiaSettingsInitialized = true;
    return;
  }
  if (!(previous.scales == runningCfg.scales)) {
    scalesInit(runningCfg.scales);
  }
}

void onManualBrewPhaseReceived(const Phase& phase) {
  if (manualProfile.phaseCount() != 1) {
    manualProfile.phases.resize(1);
  }
  manualProfile.phases[0] = phase;
}

void onUpdateSystemStateCommandReceived(const UpdateSystemStateComand& command) {
  systemState.operationMode = command.operationMode;
  systemState.tarePending = systemState.tarePending || command.tarePending;
}

void onBoilerSettingsReceived(const BoilerSettings& boilerSettings) {
  runningCfg.boiler = boilerSettings;
}

void onLedSettingsReceived(const LedSettings& ledSettings) {
  runningCfg.led = ledSettings;
}

void onSystemSettingsReceived(const SystemSettings& systemSettings) {
  runningCfg.system = systemSettings;
}

void onBrewSettingsReceived(const BrewSettings& brewSettings) {
  runningCfg.brew = brewSettings;
}

void onTareCommandReceived() {
  LOG_VERBOSE("Tare scales");
  if (currentState.scalesPresent) systemState.tarePending = true;
}

//#############################################################################################
//###############################____PROFILING_CONTROL____#####################################
//#############################################################################################

static void profiling(void) {
  if (currentState.brewActive) { //runs this only when brew button activated and pressure profile selected
    uint32_t timeInShot = millis() - brewingTimer;
    phaseProfiler.setProfile(systemState.operationMode == OperationMode::BREW_AUTO ? activeProfile : manualProfile);
    phaseProfiler.updatePhase(timeInShot, currentState);
    const CurrentPhase& currentPhase = phaseProfiler.getCurrentPhase();

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
  if (millis() - iddleTimer > 1000 * 60 * 90){ //safety if machine is forgotten on
    setBoilerOff();
    digitalWrite(shutdownPin, LOW);
  }
  // Keep that water at temp
  // TODO: If active phase overrides the water temperature, then send the active phase's temp
  justDoCoffee(runningCfg, currentState, activeProfile.waterTemperature);
}

static void manualFlowControl(void) {
  if (currentState.brewActive) {
    float flow_reading = lcdGetManualFlowVol() / 10.f ;
    setPumpFlow(flow_reading, 0.f, currentState);
    // uint32_t timeInShot = millis() - brewingTimer;
    // ShotSnapshot dummySnapshot = ShotSnapshot {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    // CurrentPhase currentPhase = CurrentPhase {0, 
    // Phase {PHASE_TYPE::PHASE_TYPE_FLOW,Transition(flow_reading, TransitionCurve::INSTANT, 0), -1, -1}, 
    // timeInShot,
    // ShotSnapshot {0, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f}};
    // ShotSnapshot shotSnapshot = buildShotSnapshot(timeInShot, currentState, currentPhase);
    // espCommsSendShotData(shotSnapshot, 100);
  } else {
    setPumpOff();
  }
  justDoCoffee(runningCfg, currentState, activeProfile.waterTemperature);
}

//#############################################################################################
//###################################____BREW DETECT____#######################################
//#############################################################################################

static void modeDetect(void) {
  currentState.brewActive = false;
  currentState.flushActive = false;
  currentState.hotWaterActive = false;

  if (systemState.operationMode == OperationMode::FLUSH_AUTO || systemState.operationMode == OperationMode::DESCALE){
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
    // needs to be here as it creates a locking state sometimes if not kept up to date during brew
    // mainly when shotWeight restriction kicks in.
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
  systemState.tarePending = true;
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
      if(currentState.steamActive) {
        lcdShowPopup("COOLDOWN");
        espCommsSendNotification(Notification::warn("COOLDOWN!"));
      } else {
        lcdShowPopup("TEMP READ ERROR");
        espCommsSendNotification(Notification::warn("TEMP READ ERROR"));
      }
      currentState.temperature  = thermocoupleRead() - runningCfg.boiler.offsetTemp;  // Making sure we're getting a value
      thermoTimer = millis() + GET_KTYPE_READ_EVERY;
    }
  }
}

static void updateStartupTimer(void) {
  systemState.timeAlive = getTimeSinceInit() / 1000;
  lcdSetUpTime(systemState.timeAlive);
}

static void cpsInit(GaggiaSettings &runningCfg) {
  int cps = getCPS();
  if (cps > 110) { // double 60 Hz
    currentState.powerLineFrequency = 60u;
  } else if (cps > 80) { // double 50 Hz
    currentState.powerLineFrequency = 50u;
  } else if (cps > 55) { // 60 Hz
    currentState.powerLineFrequency = 60u;
  } else if (cps > 0) { // 50 Hz
    currentState.powerLineFrequency = 50u;
  }
}

