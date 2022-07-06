#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/******************************************************************************/
//                       Default Serial settings
/******************************************************************************/
//set to true for debug output, false for no debug output
#define DEBUGGING true
#define DEBUG_SERIAL \
  if (DEBUGGING) Serial

#define BAUD                          115200
// #define TESTING
#ifdef DEBUGGING
#ifdef TESTING
#include "tests.h"
#endif
#endif

/*******************************************************************************
 *                         EEPROM 
 * ****************************************************************************/
const uint16_t EEPROM_SIZE          = 400;
const uint8_t  WIFI_CRED_BUF_LEN    = 29;   // Max lenght of WiFi SSID or PW
const uint16_t SSID_ADDR            = 100;  // Size = WIFI_CRED_BUF_LEN
const uint16_t KEY_ADDR             = 130;  // Size = WIFI_CRED_BUF_LEN
const uint16_t WIFI_CRED_END_ADDR   = 160;
const uint16_t LOCATION_ADDR        = 170;  // Size = 12 (3 x uint32_t)
const uint16_t LOCATION_END_ADDR    = 175;
const uint16_t SURVEY_ENABLED_ADDR  = 180;  // Size = 4
const uint16_t SURVEY_ACCURACY_ADDR = 185;  // Size = 4
const uint16_t END_ADDR             = 400;
/*******************************************************************************
 *                         WiFi settings
 * ****************************************************************************/
#define DEVICE_TYPE_PREFIX           "RTKBase_"
#define DEFAULT_KEY                  "12345678"
String getDeviceName(const String &);
uint32_t getChipId(void);

/*******************************************************************************
 *                         Default RTK settings 
 * ****************************************************************************/
#define RTK_I2C_ADDR                  0x42
// #define RTK_SDA_PIN                 33
// #define RTK_SCL_PIN                 32
#define I2C_FREQUENCY_100K            100000  // 100 kHz
#define I2C_FREQUENCY_400K            400000  // 400 kHz
// // ! see secrets.h for default location data
struct HighPrecicionLocation;  

/*******************************************************************************
 *                         Oled
 * ****************************************************************************/

/*******************************************************************************
 *                         FreeRTOS
 * ****************************************************************************/
#define RUNNING_CORE_0                0  // Low level WiFi code runs on core 0
#define RUNNING_CORE_1                1  // Use core 1 for all other tasks
// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
// where configMAX_PRIORITIES is defined within FreeRTOSConfig.h.
#define GNSS_OVER_WIFI_PRIORITY       2  // GNSS should have a lower priority
#define RTK_TASK_INTERVAL_MS          10

/*******************************************************************************
 *                         Help Functions
 * ****************************************************************************/


#endif /*** HTRTK_CONFIG_H ***/






