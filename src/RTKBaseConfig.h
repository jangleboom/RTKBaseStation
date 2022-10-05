#ifndef RTK_BASE_CONFIG_H
#define RTK_BASE_CONFIG_H
#include <Arduino.h>

// Deactivate brown out detection
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

/*
=================================================================================
                            Serial settings
=================================================================================
*/
//set to true for debug output, false for no debug output
#define DEBUGGING true
#define DBG \
  if (DEBUGGING) Serial

#define BAUD                          115200

/*
=================================================================================
                              WiFi settings
=================================================================================
*/
const uint8_t MAX_SSIDS = 10; // Space to scan and remember SSIDs

/*
=================================================================================
                              RTK settings
=================================================================================
*/
#define COORD_PRECISION               9
#define ALT_PRECISION                 4
#define RTK_I2C_ADDR                  0x42
// #define RTK_SDA_PIN                 33
// #define RTK_SCL_PIN                 32
#define CONNECTION_TIMEOUT_MS         10000   // Shorter timeouts lead to complaining email and ban
#define I2C_FREQUENCY_100K            100000  // 100 kHz
#define I2C_FREQUENCY_400K            400000  // 400 kHz
#define AUTO_SAVE_LOCATION            false   /* Save location automatically, \
but this is not longtime tested, it could lead to accumulating biases, do not change this to true yet */

/*
=================================================================================
                              Oled settings
=================================================================================
*/
#define OLED_I2C_ADDR 0x3c
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// #define SDA_PIN 4
// #define SCL_PIN 5
#define OLED_RESET -1   //   QT-PY / XIAO

/*
=================================================================================
                          FreeRTOS settings
=================================================================================
*/
#define RUNNING_CORE_0                0  // Low level WiFi code runs on core 0
#define RUNNING_CORE_1                1  // Use core 1 for all other tasks
// Each task is assigned a priority from 0 to ( configMAX_PRIORITIES - 1 ), 
// where configMAX_PRIORITIES is defined within FreeRTOSConfig.h.
#define GNSS_PRIORITY                 1
#define RTK_TASK_INTERVAL_MS          10


#endif /*** RTK_BASE_CONFIG_H ***/






