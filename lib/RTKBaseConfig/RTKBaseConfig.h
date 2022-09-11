#ifndef RTK_BASE_CONFIG_H
#define RTK_BASE_CONFIG_H
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


/*******************************************************************************
 *                         SPIFFS 
 * ****************************************************************************/

/*******************************************************************************
 *                         WiFi settings
 * ****************************************************************************/
const uint8_t MAX_SSIDS = 10; // Space to scan and remember SSIDs
// /**
//  * @brief Get the Device Name object
//  * 
//  * @return String Device Name
//  */
// String getDeviceName(const String &);

// /**
//  * @brief Get the Chip Id object
//  * 
//  * @return uint32_t Chip Id
//  */
// uint32_t getChipId(void);
#define DEVICE_NAME "rtkbase"

/*******************************************************************************
 *                         Default RTK settings 
 * ****************************************************************************/
#define RTK_I2C_ADDR                  0x42
// #define RTK_SDA_PIN                 33
// #define RTK_SCL_PIN                 32
#define I2C_FREQUENCY_100K            100000  // 100 kHz
#define I2C_FREQUENCY_400K            400000  // 400 kHz

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
#define WIFI_PRIORITY                 1  // WiFi connection
#define GNSS_PRIORITY                 2  // GNSS should have a lower priority
#define RTK_TASK_INTERVAL_MS          10
#define WIFI_TASK_INTERVAL_MS         30000
#define CHECK_WIFI_INTERVAL_MS        30000
/*******************************************************************************
 *                         Help Functions
 * ****************************************************************************/


#endif /*** RTK_BASE_CONFIG_H ***/






