/* 09:32 15/03/2023 - change triggering comment */
#include "eeprom_data.h"
#include <FlashStorage_STM32.hpp>
#include "proto/settings_converters.h"
#include "proto/proto_serializer.h"
#include "default_settings.h"
#include "../log.h"


#define MAX_PROFILES 5

namespace eeprom {
  struct eepromMetadata_t metadata;
  struct eepromValues_t   values;
  struct eepromProfiles_t profiles;
}

bool validateSettings(const GaggiaSettings newGaggiaSettings);
bool validateProfiles(const ProfileSettings newProfileSettings);
bool eepromWrite(const GaggiaSettings newGaggiaSettings, const ProfileSettings newProfileSettings) {
  if (!validateSettings(newGaggiaSettings)) return false;
  if (!validateProfiles(newProfileSettings)) return false;

  /* Serializing and saving the settings and versioning metadata */
  eeprom::values.values = newGaggiaSettings;
  eeprom::profiles.profiles = newProfileSettings;
  std::vector<uint8_t> dataInBytes = ProtoSerializer::serialize<GaggiaSettingsConverter>(eeprom::values.values);
  std::vector<uint8_t> profileInBytes = ProtoSerializer::serialize<ProfileSettingsConverter>(eeprom::profiles.profiles);

  eeprom::metadata.version = EEPROM_DATA_VERSION;
  eeprom::metadata.timestamp = millis();
  eeprom::metadata.dataLength = dataInBytes.size();
  eeprom::metadata.profileLength = profileInBytes.size();
  eeprom::metadata.versionTimestampXOR = eeprom::metadata.timestamp ^ eeprom::metadata.version;

  // Perform the data transfer for the metadata and the settings
  size_t metaDataSize = sizeof(eeprom::metadata);
  EEPROM.put(0, eeprom::metadata);

  for (size_t i = 0; i < eeprom::metadata.dataLength; i++) {
    eeprom_buffered_write_byte(metaDataSize + i, dataInBytes[i]);
  }
  for (size_t i = 0; i < eeprom::metadata.profileLength; i++) {
    eeprom_buffered_write_byte(metaDataSize + eeprom::metadata.dataLength + i, profileInBytes[i]);
  }
  eeprom_buffer_flush();

  return true;
}

bool loadCurrentEepromData() {
  EEPROM.get(0, eeprom::metadata);
  size_t metaDataSize = sizeof(eeprom::metadata);

  std::vector<uint8_t> serializedSettings;
  serializedSettings.resize(eeprom::metadata.dataLength);

  std::vector<uint8_t> serializedProfiles;
  serializedProfiles.resize(eeprom::metadata.profileLength);

  for (size_t i = 0; i < eeprom::metadata.dataLength; i++) {
    serializedSettings[i] = EEPROM.read(metaDataSize + i);
  }

  for (size_t i = 0; i < eeprom::metadata.profileLength; i++) {
    serializedProfiles[i] = EEPROM.read(metaDataSize + eeprom::metadata.dataLength + i);
  }

  eeprom::values.values = {}; // This is important to make sure we pass an empty state for deserialization
  eeprom::profiles.profiles = {}; // This is important to make sure we pass an empty state for deserialization
  return ProtoSerializer::deserialize<GaggiaSettingsConverter>(serializedSettings, eeprom::values.values) &&
         ProtoSerializer::deserialize<ProfileSettingsConverter>(serializedProfiles, eeprom::profiles.profiles);
}

void eepromInit(void) {
  eeprom::values.values = eepromGetDefaultSettings();
  eeprom::profiles.profiles = eepromGetDefaultProfiles();

  // read version
  uint16_t version;
  EEPROM.get(0, version);

  // load appropriate version (including current)
  bool readSuccess = false;

  if (version == EEPROM_DATA_VERSION) {
    readSuccess = loadCurrentEepromData();
  }

  if (!readSuccess) {
    LOG_ERROR("SECU_CHECK FAILED! Applying defaults! eeprom::metadata.version=%d", version);
  }

  if (!readSuccess || version != EEPROM_DATA_VERSION) {
    eepromWrite(eeprom::values.values, eeprom::profiles.profiles);
  }
}

GaggiaSettings eepromGetCurrentSettings(void) {
  return eeprom::values.values;
}

GaggiaSettings eepromGetDefaultSettings(void) {
  return getDefaultGaggiaSettings();
}

ProfileSettings eepromGetCurrentProfiles(void) {
  return eeprom::profiles.profiles;
}

ProfileSettings eepromGetDefaultProfiles(void) {
  return getDefaultProfileSettings();
}

/* Spot-check various settings values */
bool validateSettings(const GaggiaSettings newGaggiaSettings) {
  const char* errMsg = "Data out of range";

  if (newGaggiaSettings.boiler.steamSetPoint < 1
    || newGaggiaSettings.boiler.steamSetPoint > 165
    || newGaggiaSettings.boiler.mainDivider < 1
    || newGaggiaSettings.boiler.brewDivider < 1
    || newGaggiaSettings.system.pumpFlowAtZero < 0.210f
    || newGaggiaSettings.system.pumpFlowAtZero > 0.310f
    || newGaggiaSettings.system.scalesF1 < -20000
    || newGaggiaSettings.system.scalesF2 > 20000)
  {
    LOG_ERROR(errMsg);
    return false;
  }

  return true;
}

bool validateProfiles(const ProfileSettings newProfileSettings) {
  const char* errMsg = "Data out of range";

  if (newProfileSettings.savedProfiles.size() > MAX_PROFILES) {
    LOG_ERROR(errMsg);
    return false;
  }

  for (auto profile : newProfileSettings.savedProfiles) {
    if (profile.name.length() == 0
      || profile.phaseCount() == 0
      || profile.phaseCount() > 8
      || profile.waterTemperature < 1)
    {
      LOG_ERROR(errMsg);
      return false;
    }
  }

  return true;
}
