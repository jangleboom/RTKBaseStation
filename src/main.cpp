
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
 *          - write func for converting lat/long hight into X/Y/Z coords
 *          - make coords input dynamically on server page
 *          - set target accuracy for survey
 *          - add display and buttons
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
        delay(1000);
        }
  }
    
  myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_RTCM3 | COM_TYPE_NMEA);  //UBX+RTCM3 is not a valid option so we enable all three.
  // myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save the communications port settings to flash and BBR
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

  if (STATIC_POSITION == true) {
    // TODO: write a func to generate that input!
    // Latitude, Longitude, Altitude input:
    response &= myGNSS.setStaticPosition(LATITUDE, LATITUDE_HP, LONGITUDE, LONGITUDE_HP, ALTITUDE, ALTITUDE_HP, true); 
    // Earth-centered corrdinates:
    //response &= myGNSS.setStaticPosition(ECEF_X_CM, ECEF_X_HP, ECEF_Y_CM, ECEF_Y_HP, ECEF_Z_CM, ECEF_Z_HP);  //With high precision 0.1mm parts
    if (response == false) {
      DEBUG_SERIAL.println(F("Failed to enter static position. Freezing..."));
      while (1)
        ;
    } else {
      DEBUG_SERIAL.println(F("Static position set"));
    }

  } else 
  {
    //Alternatively to setting a static position, you could do a survey-in
    //but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
    // Check if Survey is in Progress before initiating one
    // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
    // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
    // You can either read the data from packetUBXNAVSVIN directly
    // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
    response &= myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
    if (response == false)
    {
      DEBUG_SERIAL.println(F("Failed to get Survey In status. Freezing."));
      while (1)
        ; //Freeze
    }

    if (myGNSS.getSurveyInActive() == true) // Use the helper function
    {
      DEBUG_SERIAL.println(F("Survey already in progress."));
      // lcd.setCursor(0, 2);
      // lcd.print(F("Survey already going"));
    }
    else
    {
      //Start survey
      response = myGNSS.enableSurveyMode(60, 5.000); //Enable Survey in, 60 seconds, 5.0m
      if (response == false)
      {
        DEBUG_SERIAL.println(F("Survey start failed"));
        // lcd.setCursor(0, 3);
        // lcd.print(F("Survey start failed. Freezing."));
        while (1)
          ;
      }
    DEBUG_SERIAL.println(F("Survey started. This will run until 60s has passed and less than 5m accuracy is achieved."));
    }

    //Begin waiting for survey to complete
    while (myGNSS.getSurveyInValid() == false) // Call the helper function
    {
      // if (Serial.available())
      // {
      //   byte incoming = Serial.read();
      //   if (incoming == 'x')
      //   {
      //     //Stop survey mode
      //     response = myGNSS.disableSurveyMode(); //Disable survey
      //     Serial.println(F("Survey stopped"));
      //     break;
      //   }
      // }

      // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
      // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
      // You can either read the data from packetUBXNAVSVIN directly
      // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
      response &= myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
      if (response == true)
      {
        Serial.print(F("Time elapsed: "));
        Serial.print((String)myGNSS.getSurveyInObservationTime()); // Call the helper function

        // lcd.setCursor(0, 1);
        // lcd.print(F("Elapsed: "));
        // lcd.print((String)myGNSS.getSurveyInObservationTime()); // Call the helper function

        Serial.print(F(" Accuracy: "));
        Serial.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
        Serial.println();

        // lcd.setCursor(0, 2);
        // lcd.print(F("Accuracy: "));
        // lcd.print((String)myGNSS.getSurveyInMeanAccuracy()); // Call the helper function
      }
      else {
        Serial.println(F("SVIN request failed"));
      }

      delay(1000);
    }
    Serial.println(F("Survey valid!"));

    Serial.println(F("Base survey complete! RTCM now broadcasting."));

    //If you were setting up a full GNSS station, you would want to save these settings.
    //Because setting an incorrect static position will disable the ability to get a lock, we will skip saving during this example
    //if (myGNSS.saveConfiguration() == false) //Save the current settings to flash and BBR
    //  DEBUG_SERIAL.println(F("Module failed to save"));
  }


   
/* 
  ECEF coordinates: Example tiny office Brieslang
  after running look here: http://new.rtk2go.com:2101/SNIP::STATUS
*/



  DEBUG_SERIAL.println(F("Module configuration complete"));
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

    bool connectionSuccess = false;
    char response[512];
    int responseSpot = 0;
    
    while (1) {
      // beginServing() func content
      // Connect if we are not already
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

            // Wait for response
            unsigned long timeout = millis();
            while (ntripCaster.available() == 0) {
            if (millis() - timeout > 5000) {
                DEBUG_SERIAL.println(F("Caster timed out!"));
                ntripCaster.stop();
                // return;
                }
            delay(10);
            }

            // Check reply
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
                // return;
                }
            }  // End attempt to connect
            else {
                DEBUG_SERIAL.println(F("Connection to host failed"));
                // return;
            }
        }  // End connected == false

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
                // return;
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