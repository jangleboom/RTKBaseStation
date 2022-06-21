
/**
 * @file WiFiManager.h
 * @author based on Roland Pelayo, refactored by Markus Hädrich
 * @link https://www.teachmemicro.com/esp32-wifi-manager-dynamic-ssid-password
 * <br>
 * @brief This is part of a distributed software, here: head tracker and GNNS positioning
 * using Sparkfun Real Time Kinematics
 * <br>
 * @todo  - a lot
 * @note no "Ä,ä, Ö, ö, Ü, ü" or other strange letters as SSID and PW allowed 
 */


#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WebServer.h>
#include "config.h"
#include "EEPROM.h"
#include "html.h"

/**
 * @brief Function to handle unknown URLs
 */
void handleNotFound(void);

/**
 * @brief Function for writing WiFi creds to EEPROM
 * @return true if save successful, false if unsuccessful
 */
bool writeToMemory(String ssid, String pass);

/**
 * @brief Function for handling form
 */
void handleSubmit(void);

/**
 * @brief Function for home page
 */
void handleRoot(void);

/**
 * @brief Function for loading form
 * @return false if no WiFi creds in EEPROM
 */
bool loadWiFiCredsForm(void);

/**
 * @brief Function checking WiFi creds in memory 
 * @return: true if not empty, false if empty
 */
bool checkWiFiCreds(void);

/**
 * @brief Wipes out the stored WIFI credentials from EEPROM
 * 
 */
void wipeEEPROM(void);

void setupAP(void);

#endif /* WIFI_MANAGER_H */ 