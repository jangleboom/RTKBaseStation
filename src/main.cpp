
/*******************************************************************************
 * @file    main.cpp
 * @authors Markus Hädrich inspired by SparkFun Electronics / Nathan Seidle
 * <br>
 * @brief   This is part of a distributed software, here: 
 *          Real Time Kinematics (RTK) base station.
 *          The ESP32 is acting as a 'server' to a 'caster'. In this case we will 
 *          use RTK2Go.com (or Emlid, look at secrets.h) as caster because it is free. 
 *          A rover can then connect to RTK2Go (or Emlid) as a 'client' and get the RTCM 
 *          data it needs. You will need to register your mountpoint here: 
 *          http://www.rtk2go.com/new-reservation/ 
 *          To see if your mountpoint is active go here: http://rtk2go.com:2101/ 
 * <br>
 * @date    2022/05/09
 * 
 * @todo    - first check if myGNSS is getting data BEFORE establish the caster connection,
 *            otherwise they will ban our IP for 4 hours minimum
 *        
 * @note    How to handle WiFi: 
 *           - Push the button 
 *           - Join the AP thats appearing 
 *               -# SSID: e. g. "RTKBase_" + ChipID 
 *               -# PW: e. g. "12345678"
 *            - Open address 192.168.4.1 in your browser and set credentials you are 
 *              using for you personal access point on your smartphone
 *            - If the process is done, the LED turns off and the device reboots
 *            - If there are no Wifi credentials stored in the EEPROM, the device 
 *              will jump in this mode on startup
 *            - Hint: If needed do your HTML changes in the index.html file, then copy the content to
 *              https://davidjwatts.com/youtube/esp8266/esp-convertHTM.html#
 *              paste the result into the html.h file
 * 
 *          How to measure battery: Easiest way, use a fuel gauge breakout board 
 *          e. g. Adafruit_LC709203F, because the Sparkfun ESP32 Thing Plus 
 *                  
 * 
 * @version 0.43
 ******************************************************************************/


#include <Arduino.h>
#include <Wire.h> // BNO080 and uBlox GNSS
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <config.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <secrets.h> // You need to create your own header file, like discribed in README.md

String deviceName = getDeviceName(DEVICE_TYPE);

WiFiClient ntripCaster;
//Global Variables
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSentRTCM_ms = 0;            //Time of last data pushed to socket
int maxTimeBeforeHangup_ms = 10000;  //If we fail to get a complete RTCM frame after 10s, then disconnect from caster

uint32_t serverBytesSent = 0;  //Just a running total
long lastReport_ms = 0;        //Time of last report of bytes sent
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

/*******************************************************************************
 *                                 Button(s)
 * ****************************************************************************/
#include "Button2.h"
// Button to press to wipe out stored WiFi credentials
const int BUTTON_PIN = 15;
Button2 button = Button2(BUTTON_PIN, INPUT, false, false);

void buttonHandler(Button2 &btn);

/*******************************************************************************
 *                                 WiFi
 * ****************************************************************************/

void setupWiFi(const String& ssid, const String& key);

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/
long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

SFE_UBLOX_GNSS myGNSS;

void setupGNSS(void);
void task_rtk_wifi_connection(void *pvParameters);

void setup() {
    #ifdef DEBUGGING
    Serial.begin(BAUD);
    while (!Serial) {};
    #endif

    DEBUG_SERIAL.print(F("Device name: "));
    DEBUG_SERIAL.println(deviceName);

    button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here  
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // TODO: make the WiFi setup a primary task
    EEPROM.begin(400);
    //wipeEEPROM();
    if (!checkWiFiCreds()) {
        digitalWrite(LED_BUILTIN, HIGH);
        DEBUG_SERIAL.println(F("No WiFi credentials stored in memory. Loading form..."));
        while (loadWiFiCredsForm());
    }  else {
    // Then log into WiFi
    String ssid = EEPROM.readString(SSID_ADDR);
    String key = EEPROM.readString(KEY_ADDR);
    setupWiFi(ssid, key);
    };

    xTaskCreatePinnedToCore( &task_rtk_wifi_connection, "task_rtk_wifi_connection", 20480, NULL, GNSS_OVER_WIFI_PRIORITY, NULL, RUNNING_CORE_0);
    
    String thisBoard= ARDUINO_BOARD;
    DEBUG_SERIAL.print(F("Setup done on "));
    DEBUG_SERIAL.println(thisBoard);
}


void loop() {
    #ifdef DEBUGGING
    #ifdef TESTING
    DEBUG_SERIAL.println(F("Running Tests..."))
    aunit::TestRunner::run();
    #endif
    #endif

    button.loop();
}

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/

void setupGNSS() {
    if (myGNSS.begin() == false) {
    DEBUG_SERIAL.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1) {
        vTaskDelay(100/portTICK_PERIOD_MS);
        }
    }
    
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3);  //UBX+RTCM3 is not a valid option so we enable all three.

  myGNSS.setNavigationFrequency(1);  //Set output in Hz. RTCM rarely benefits from >1Hz.

    //Disable all NMEA sentences
  bool response = true;
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GGA, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSA, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GSV, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_RMC, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GST, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_GLL, COM_PORT_I2C);
  response &= myGNSS.disableNMEAMessage(UBX_NMEA_VTG, COM_PORT_I2C);

  if (response == false) {
    DEBUG_SERIAL.println(F("Failed to disable NMEA. Freezing..."));
    while (1)
      ;
  } else
    DEBUG_SERIAL.println(F("NMEA disabled"));

  //Enable necessary RTCM sentences
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1005, COM_PORT_I2C, 1);  //Enable message 1005 to output through UART2, message every second
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1074, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1084, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1094, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1124, COM_PORT_I2C, 1);
  response &= myGNSS.enableRTCMmessage(UBX_RTCM_1230, COM_PORT_I2C, 10);  //Enable message every 10 seconds

  if (response == false) {
    DEBUG_SERIAL.println(F("Failed to enable RTCM. Freezing..."));
    while (1)
      ;
  } else
    DEBUG_SERIAL.println(F("RTCM sentences enabled"));

  //-1280208.308,-4716803.847,4086665.811 is SparkFun HQ so...
  //Units are cm with a high precision extension so -1234.5678 should be called: (-123456, -78)
  //For more infomation see Example12_setStaticPosition
  //Note: If you leave these coordinates in place and setup your antenna *not* at SparkFun, your receiver
  //will be very confused and fail to generate correction data because, well, you aren't at SparkFun...
  //See this tutorial on getting PPP coordinates: https://learn.sparkfun.com/tutorials/how-to-build-a-diy-gnss-reference-station/all
  // response &= myGNSS.setStaticPosition(-128020830, -80, -471680384, -70, 408666581, 10);  //With high precision 0.1mm parts
  response &= myGNSS.setStaticPosition(377515946, 60, 493793830, 30, 142561157, 10);  //With high precision 0.1mm parts
  if (response == false) {
    DEBUG_SERIAL.println(F("Failed to enter static position. Freezing..."));
    while (1)
      ;
  } else
    DEBUG_SERIAL.println(F("Static position set"));
/*ECEF coordinates: Example Brieslang
Get LLH coords from map: https://www.gpskoordinaten.de/ Breitengrad: 52.601314 | Längengrad: 13.001833 | Höhe: 39 Meter
Convert LLH coords into ECEF XYZ coords: https://tool-online.com/en/coordinate-converter.php
WGS84_XYZ (geocentric)  ECEF-X 3775159.466
WGS84_XYZ (geocentric)  ECEF-Y 4937938.303
WGS84_XYZ (geocentric)  ECEF-Z 1425611.571

after running look here: http://new.rtk2go.com:2101/SNIP::STATUS
*/

  //Alternatively to setting a static position, you could do a survey-in
  //but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
  //myGNSS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m

  //If you were setting up a full GNSS station, you would want to save these settings.
  //Because setting an incorrect static position will disable the ability to get a lock, we will skip saving during this example
  //if (myGNSS.saveConfiguration() == false) //Save the current settings to flash and BBR
  //  DEBUG_SERIAL.println(F("Module failed to save"));

  DEBUG_SERIAL.println(F("Module configuration complete"));
}

void beginServing() {
    //Connect if we are not already
    if (ntripCaster.connected() == false) {
      DEBUG_SERIAL.printf("Opening socket to %s\n", casterHost);

      if (ntripCaster.connect(casterHost, casterPort) == true)  //Attempt connection
      {
        DEBUG_SERIAL.printf("Connected to %s:%d\n", casterHost, casterPort);

        const int SERVER_BUFFER_SIZE = 512;
        char serverRequest[SERVER_BUFFER_SIZE];

        snprintf(serverRequest,
                 SERVER_BUFFER_SIZE,
                 "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun u-blox Server v1.0\r\n\r\n",
                 mountPointPW, mountPoint);

        DEBUG_SERIAL.println(F("Sending server request:"));
        DEBUG_SERIAL.println(serverRequest);
        ntripCaster.write(serverRequest, strlen(serverRequest));

        //Wait for response
        unsigned long timeout = millis();
        while (ntripCaster.available() == 0) {
          if (millis() - timeout > 5000) {
            DEBUG_SERIAL.println(F("Caster timed out!"));
            ntripCaster.stop();
            return;
          }
          vTaskDelay(10);
        }

        //Check reply
        bool connectionSuccess = false;
        char response[512];
        int responseSpot = 0;
        while (ntripCaster.available()) {
          response[responseSpot++] = ntripCaster.read();
          if (strstr(response, "200") > 0)  //Look for 'ICY 200 OK'
            connectionSuccess = true;
          if (responseSpot == 512 - 1)
            break;
        }
        response[responseSpot] = '\0';

        if (connectionSuccess == false) {
          DEBUG_SERIAL.printf("Failed to connect to Caster: %s", response);
          return;
        }
      }  //End attempt to connect
      else {
        DEBUG_SERIAL.println(F("Connection to host failed"));
        return;
      }
    }  //End connected == false

    lastReport_ms = millis();
    lastSentRTCM_ms = millis();

    //This is the main sending loop. We scan for new ublox data but processRTCM() is where the data actually gets sent out.
    while (ntripCaster.connected() == true) {
        myGNSS.checkUblox();  //See if new data is available. Process bytes as they come in.

        //Close socket if we don't have new data for 10s
        //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
        //So let's not leave the socket open/hanging without data
        if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms) {
            DEBUG_SERIAL.println(F("RTCM timeout. Disconnecting..."));
            ntripCaster.stop();
    }

    vTaskDelay(10);

    //Report some statistics every 250
    if (millis() - lastReport_ms > 250) {
        lastReport_ms += 250;
        DEBUG_SERIAL.printf("Total sent: %d\n", serverBytesSent);
        }
    }
}

// This function gets called from the SparkFun u-blox Arduino Library.
// As each RTCM byte comes in you can specify what to do with it
// Useful for passing the RTCM correction data to a radio, Ntrip broadcaster, etc.
void SFE_UBLOX_GNSS::processRTCM(uint8_t incoming) {
    if (ntripCaster.connected() == true) {
    ntripCaster.write(incoming);  //Send this byte to socket
    serverBytesSent++;
    lastSentRTCM_ms = millis();
  }
}
/*******************************************************************************
 *                                 WiFi
 * ****************************************************************************/

void setupWiFi(const String& ssid, const String& key) {
    delay(10);
    // Connecting to a WiFi network
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print(F("Connecting to "));
    DEBUG_SERIAL.println(ssid);
    WiFi.mode(WIFI_STA);
    WiFi.softAPdisconnect(true);
    WiFi.begin(ssid.c_str(), key.c_str());
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        DEBUG_SERIAL.print(F("."));
    }
    DEBUG_SERIAL.println(F(""));
    DEBUG_SERIAL.println(F("WiFi connected"));
    DEBUG_SERIAL.println(F("IP address: "));
    DEBUG_SERIAL.println(WiFi.localIP());
}

void task_rtk_wifi_connection(void *pvParameters) {
    (void)pvParameters;
    Wire.begin();
    Wire.setClock(I2C_FREQUENCY_100K);
    setupGNSS();
    // Measure stack size
    // UBaseType_t uxHighWaterMark; 
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_rtk_wifi_connection setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);
    
    while (1) {
        // beginServing();
            //Connect if we are not already
        if (ntripCaster.connected() == false) {
            DEBUG_SERIAL.printf("Opening socket to %s\n", casterHost);

        if (ntripCaster.connect(casterHost, casterPort) == true)  //Attempt connection
        {
            DEBUG_SERIAL.printf("Connected to %s:%d\n", casterHost, casterPort);

            const int SERVER_BUFFER_SIZE = 512;
            char serverRequest[SERVER_BUFFER_SIZE];

            snprintf(serverRequest,
                    SERVER_BUFFER_SIZE,
                    "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun u-blox Server v1.0\r\n\r\n",
                    mountPointPW, mountPoint);

            DEBUG_SERIAL.println(F("Sending server request:"));
            DEBUG_SERIAL.println(serverRequest);
            ntripCaster.write(serverRequest, strlen(serverRequest));

            //Wait for response
            unsigned long timeout = millis();
            while (ntripCaster.available() == 0) {
            if (millis() - timeout > 5000) {
                DEBUG_SERIAL.println(F("Caster timed out!"));
                ntripCaster.stop();
                return;
                }
            delay(10);
            }

            //Check reply
            bool connectionSuccess = false;
            char response[512];
            int responseSpot = 0;
            while (ntripCaster.available()) {
            response[responseSpot++] = ntripCaster.read();
            if (strstr(response, "200") > 0)  //Look for 'ICY 200 OK'
                connectionSuccess = true;
            if (responseSpot == 512 - 1)
                break;
            }
            response[responseSpot] = '\0';

            if (connectionSuccess == false) {
                DEBUG_SERIAL.printf("Failed to connect to Caster: %s", response);
                return;
                }
            }  //End attempt to connect
            else {
                DEBUG_SERIAL.println(F("Connection to host failed"));
                return;
            }
        }  //End connected == false

        lastReport_ms = millis();
        lastSentRTCM_ms = millis();

        //This is the main sending loop. We scan for new ublox data but processRTCM() is where the data actually gets sent out.
        while (ntripCaster.connected() == true) {
            myGNSS.checkUblox();  //See if new data is available. Process bytes as they come in.

            //Close socket if we don't have new data for 10s
            //RTK2Go will ban your IP address if you abuse it. See http://www.rtk2go.com/how-to-get-your-ip-banned/
            //So let's not leave the socket open/hanging without data
            if (millis() - lastSentRTCM_ms > maxTimeBeforeHangup_ms) {
                DEBUG_SERIAL.println(F("RTCM timeout. Disconnecting..."));
                ntripCaster.stop();
                return;
        }

        delay(10);

        //Report some statistics every 250
        if (millis() - lastReport_ms > 250) {
            lastReport_ms += 250;
            DEBUG_SERIAL.printf("Total sent: %d\n", serverBytesSent);
            }
        }

        // Measure stack size (last was 17772)
        // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        // DEBUG_SERIAL.print(F("task_rtk_wifi_connection loop, uxHighWaterMark: "));
        // DEBUG_SERIAL.println(uxHighWaterMark);
        vTaskDelay(RTK_TASK_INTERVAL_MS/portTICK_PERIOD_MS);
    }
    // Delete self task
    ntripCaster.stop();
    vTaskDelete(NULL);
}


/*******************************************************************************
 *                                 Further system components
 * ****************************************************************************/


void buttonHandler(Button2 &btn) 
{
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("Wiping WiFi credentials from memory..."));
    wipeEEPROM();
    while (loadWiFiCredsForm()) {};
  }
}