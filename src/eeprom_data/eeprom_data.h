/* 09:32 15/03/2023 - change triggering comment */
#ifndef EEPROM_DATA_H
#define EEPROM_DATA_H

#include <Arduino.h>
#include "gaggia_settings.h"

#define EEPROM_DATA_VERSION 13

// BaseMetadata class. Used to support loading potentially different versions of metadata
struct BaseMetadata_t {
  uint16_t version;
  unsigned long timestamp;
};


// BaseValue class extended by all value structs to support migrations from different versions.
struct BaseValues_t {};

struct eepromMetadata_t: public BaseMetadata_t {
  size_t dataLength;
  size_t profileLength;
  uint32_t versionTimestampXOR;
};

struct eepromValues_t: public BaseValues_t {
  GaggiaSettings values;
};
struct eepromProfiles_t: public BaseValues_t {
  ProfileSettings profiles;
};

void eepromInit(void);
bool eepromWrite(const GaggiaSettings newGaggiaSettings, const ProfileSettings newGaggiaProfiles);
GaggiaSettings eepromGetDefaultSettings(void);
GaggiaSettings eepromGetCurrentSettings(void);
ProfileSettings eepromGetCurrentProfiles(void);
ProfileSettings eepromGetDefaultProfiles(void);

#define ACTIVE_PROFILE(profiles) profiles.savedProfiles[profiles.activeProfileIndex]

#endif
