#ifndef GAGGIA_DEFAULT_SETTINGS_H
#define GAGGIA_DEFAULT_SETTINGS_H

#include "gaggia_settings.h"
#include "new_default_profiles.h"

GaggiaSettings getDefaultGaggiaSettings(void) {
  GaggiaSettings defaultData;

  // Profiles
  defaultData.profiles.activeProfileIndex = 0;
  defaultData.profiles.savedProfiles.clear();
  for (auto profile : defaultProfiles) {
    defaultData.profiles.savedProfiles.push_back(Profile(profile));
  }

  // Boiler
  defaultData.boiler.steamSetPoint = 165;
  defaultData.boiler.offsetTemp = 3;
  defaultData.boiler.hpwr = 10;
  defaultData.boiler.mainDivider = 70;
  defaultData.boiler.brewDivider = 30;

  // Screen
  defaultData.brew.homeOnShotFinish = true;
  defaultData.brew.brewDeltaState = false;
  defaultData.brew.basketPrefill = false;

  // System settings
  defaultData.system.powerLineFrequency = 60;
  defaultData.system.scalesF1 = 1010;
  defaultData.system.scalesF2 = 1010;
  defaultData.system.pumpFlowAtZero = 0.2401f;
  defaultData.system.lcdSleep = 16;
  defaultData.system.warmupState = false;

  // LED settings
  defaultData.led.state = true;
  defaultData.led.color.R = 9;
  defaultData.led.color.G = 0;
  defaultData.led.color.B = 9;

  return defaultData;
}

#endif
