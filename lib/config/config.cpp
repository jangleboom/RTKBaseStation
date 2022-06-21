#include "config.h"

struct HighPrecisionLocation {
  uint32_t magic;                 // Magic keyword, that tells us if we're reading garbage from EEPROM
  int32_t lat;                    // 7-digits
  int8_t lat_high_pre_part;       // high precision extension
  int32_t lon;                    // 7-digits
  int8_t lon_high_prec_part;      // high precision extension
  int32_t alt;                    // Unit: mm
  int8_t alt_high_prec_part;      // high precision extension
};

uint32_t getChipId() {
  uint32_t chipId = 0;

  for(int i=0; i<17; i=i+8) {
	  chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
    DEBUG_SERIAL.print("chipId: ");
    DEBUG_SERIAL.println(chipId, HEX);
    return chipId;
}

String getDeviceName(const String &prefix) 
{
  unsigned int prefixLen = prefix.length();
  unsigned int suffixLen = 6;
  
  String deviceName((char *)0);
  String suffix = String(getChipId(), HEX);
  suffix.toUpperCase();
  
  deviceName.reserve(prefixLen + suffixLen);
  deviceName += prefix;
  deviceName += suffix;

  return deviceName;
}