
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
 * @todo    - Read SPIFFS settings before setup GNSS
 *          - Set AP IP
 *          - first check if myGNSS is getting data BEFORE establish the caster connection,
 *            otherwise they will ban our IP for 4 hours minimum
 *          - write func for converting lat/long hight into X/Y/Z coords
 *          - add display and buttons
 *          - save location after survey is finished
 *          - check input format of coordinates on html
 *          - reset after updating web form content, accuracy and location ect.
 *          - check if location data is saved on memory, if yes - dont make a survey and use them to set static location
 * 
 * @link https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library/blob/main/examples/ZED-F9P/Example4_BaseWithLCD/Example4_BaseWithLCD.ino
 *        
 * @note    How to handle WiFi: 
 *           - Push the button 
 *           - Join the AP thats appearing 
 *               -# SSID: e. g. "RTKBase" 
 *               -# PW: e. g. "12345678"
 *            - Open address 192.168.4.1 in your browser and set credentials you are 
 *              using for you personal access point on your smartphone
 *            - If the process is done, the LED turns off and the device reboots
 *            - If there are no Wifi credentials stored in the EEPROM, the device 
 *              will jump in WIFI_AP mode on startup, if yes it tries to connect in WIFI_STA to the 
 *              saved SSID
 * 
 *          How to measure battery: Easiest way, use a fuel gauge breakout board 
 *          e. g. Adafruit_LC709203F, because the Sparkfun ESP32 Thing Plus 
 *                  
 * 
 * @version 0.42
 ******************************************************************************/


#include <Arduino.h>
#include <Wire.h> // Display and uBlox GNSS
#include <RTKBaseConfig.h>
#include <RTKSecrets.h> // You need to create your own header file, like discribed in README.md

/*******************************************************************************
 *                                 Display
 * ****************************************************************************/
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#define OLED_I2C_ADDR 0x3c
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
// #define SDA_PIN 4
// #define SCL_PIN 5
#define OLED_RESET -1   //   QT-PY / XIAO
// Global objects
Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
bool displayConnected;
// Prototypes
bool setupDisplay(void);


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
#include <SPIFFS.h>
#include <RTKBaseManager.h>
#include <WiFi.h>

using namespace RTKBaseManager;
AsyncWebServer server(80);


String scannedSSIDs[MAX_SSIDS];

// Globals
WiFiClient ntripCaster;

/*******************************************************************************
 *                                 GNSS
 * ****************************************************************************/
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// ? Avoid Global Variables
long lastSentRTCM_ms = 0;             // Time of last data pushed to socket
int maxTimeBeforeHangup_ms = 10000;   /* If we fail to get a complete RTCM frame after 10s, 
                                          then disconnect from caster */
uint32_t serverBytesSent = 0;         // Just a running total
long lastReport_ms = 0;               // Time of last report of bytes sent

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
// Globals
SFE_UBLOX_GNSS myGNSS;

void setupRTKBase(bool surveyEnabled);
float getDesiredSurveyAccuracy(const char* path);
void runSurvey(float desiredAccuracyInM, bool resp);
double getLongitudeHp(void);
double getLatitudeHp(void);
// double getHeightOverSeaLevelHp(void);
float getHeightOverSeaLevel(void);
float getAccuracy(void);
bool saveLocation(void);
void printHighPrecisionPositionAndAccuracy(void);
void task_rtk_wifi_connection(void *pvParameters);

// Help funcs
String secondsToTimeFormat(uint32_t sec);


void setup() {
  Wire.begin();
  #ifdef DEBUGGING
  Serial.begin(BAUD);
  while (!Serial) {};
  #endif
  
  
  // //Disable survey
  // DEBUG_SERIAL.print(F("Survey stopped: "));
  // DEBUG_SERIAL.println(myGNSS.disableSurveyMode());

  // myGNSS.disableSurveyMode();  // Call if RTK device should be restarted on ESP32 start
  // Initialize SPIFFS, set true for formatting (at first time running is a must)
  bool format = false;
  if (!RTKBaseManager::setupSPIFFS(format)) {
    DEBUG_SERIAL.println(F("setupSPIFFS failed, freezing"));
    while (true) {};
  }

  DEBUG_SERIAL.print(F("Device name: "));DEBUG_SERIAL.println(DEVICE_NAME);

  String locationMethod = readFile(SPIFFS, PATH_RTK_LOCATION_METHOD);
  DEBUG_SERIAL.print(F("Location method: ")); DEBUG_SERIAL.println(locationMethod);
  
  location_int_t lastLocation;
  if (getIntLocationFromSPIFFS(&lastLocation, PATH_RTK_LOCATION_LATITUDE, PATH_RTK_LOCATION_LONGITUDE, PATH_RTK_LOCATION_ALTITUDE)) {
    printIntLocation(&lastLocation);
  }

  button.setPressedHandler(buttonHandler); // INPUT_PULLUP is set too here  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  WiFi.setHostname(DEVICE_NAME);
  // Check if we have credentials for a available network
  String lastSSID = readFile(SPIFFS, PATH_WIFI_SSID);
  String lastPassword = readFile(SPIFFS, PATH_WIFI_PASSWORD);

  if (!savedNetworkAvailable(lastSSID) || lastPassword.isEmpty() ) {
    setupAPMode(AP_SSID, AP_PASSWORD);
    delay(500);
  } else {
   setupStationMode(lastSSID.c_str(), lastPassword.c_str(), DEVICE_NAME);
   delay(500);
 }
  startServer(&server);
  
  setupDisplay();

  xTaskCreatePinnedToCore( &task_rtk_wifi_connection, "task_rtk_wifi_connection", 20480, NULL, GNSS_OVER_WIFI_PRIORITY, NULL, RUNNING_CORE_0);
  
  String thisBoard = ARDUINO_BOARD;
  DEBUG_SERIAL.print(F("Setup done on "));
  DEBUG_SERIAL.println(thisBoard);
}


void loop() {
    // #ifdef DEBUGGING
    // DEBUG_SERIAL.println(F("Running Tests..."));
    // aunit::TestRunner::run();
    // #endif
    button.loop();
}

/*******************************************************************************
 *                                 RTK/GNSS
 * ****************************************************************************/

float getDesiredSurveyAccuracy(const char* path) {
  String savedAccuray = readFile(SPIFFS, path);
  if (savedAccuray.isEmpty()) {
    return DESIRED_ACCURACY_M;
  } else {
    return savedAccuray.toFloat();
  }
}

void runSurvey(float desiredAccuracyInM, bool resp) {
    //Alternatively to setting a static position, you could do a survey-in
    //but it takes much longer to start generating RTCM data. See Example4_BaseWithLCD
    // Check if Survey is in Progress before initiating one
    // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
    // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
    // You can either read the data from packetUBXNAVSVIN directly
    // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
    bool response = resp;
    response &= myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (request can take a long time)
    if (response == false)
    {
      DEBUG_SERIAL.println(F("Failed to get Survey In status. Freezing."));
      while (1)
        ; //Freeze
    }

    if (myGNSS.getSurveyInActive() == true) // Use the helper function
    {
      const String status = "Survey already going";
      float meanAccuracy = myGNSS.getSurveyInMeanAccuracy();
      DEBUG_SERIAL.println(status);
      if (displayConnected) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("SSID: ");
        display.print(WiFi.SSID());
        display.setCursor(0, 10);
        display.print("IP: ");
        display.print(WiFi.localIP());
        display.setCursor(0, 20);
        display.print("http://"); display.print(DEVICE_NAME); display.print(".local");
        display.setCursor(0, 40);
        display.print(status);
        display.setCursor(0, 50);
        display.print(F("Acc.: "));
        display.print(String(meanAccuracy)); // Call the helper function
        display.print(" -> ");
        display.print(String(desiredAccuracyInM)); // Call the helper function
        display.print(F(" m"));
        display.display();
      }
    }
    else
    {
      //Start survey
      response = myGNSS.enableSurveyModeFull(60, desiredAccuracyInM); //Enable Survey in, 60 seconds, 1.0m
      // response = myGNSS.enableSurveyMode(60, desiredAccuracyInM); //Enable Survey in, 60 seconds, desiredAccuracyInM (m)
      if (response == false)
      {
        const String status = "Survey start failed";
        DEBUG_SERIAL.println(status);
        if (displayConnected) {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print(DEVICE_NAME);
          display.setCursor(0, 20);
          display.print(status);
          display.setCursor(0, 40);
          display.print(F("Freezing..."));
          display.display();
        }
   
        while (1)
          ;
      }
    DEBUG_SERIAL.print(F("Survey started. This will run until 60s has passed and less than "));
    DEBUG_SERIAL.print(desiredAccuracyInM);
    DEBUG_SERIAL.println(F(" mm accuracy is achieved."));
    }

    //Begin waiting for survey to complete
    while (myGNSS.getSurveyInValid() == false) // Call the helper function
    {
      // From v2.0, the data from getSurveyStatus (UBX-NAV-SVIN) is returned in UBX_NAV_SVIN_t packetUBXNAVSVIN
      // Please see u-blox_structs.h for the full definition of UBX_NAV_SVIN_t
      // You can either read the data from packetUBXNAVSVIN directly
      // or can use the helper functions: getSurveyInActive; getSurveyInValid; getSurveyInObservationTime; and getSurveyInMeanAccuracy
      response &= myGNSS.getSurveyStatus(2000); //Query module for SVIN status with 2000ms timeout (req can take a long time)
      if (response == true)
      {
        uint32_t timeElapsed = myGNSS.getSurveyInObservationTime();
        float meanAccuracy = myGNSS.getSurveyInMeanAccuracy();

        DEBUG_SERIAL.print(F("Time elapsed: "));
        DEBUG_SERIAL.print((String)myGNSS.getSurveyInObservationTime()); // Call the helper function
        DEBUG_SERIAL.print(F(" Accuracy: "));
        DEBUG_SERIAL.println(meanAccuracy); // Call the helper function

        if (displayConnected) {
          display.clearDisplay();
          display.setCursor(0, 0);
          display.print("SSID: ");
          display.print(WiFi.SSID());
          display.setCursor(0, 10);
          display.print("IP: ");
          display.print(WiFi.localIP());
          display.setCursor(0, 20);
          display.print("http://");display.print(DEVICE_NAME);display.print(".local");
          display.setCursor(0, 30);
          display.print("Survey: ");
          display.print(secondsToTimeFormat(timeElapsed)); // Call the helper function
          display.setCursor(0, 40);
          display.print(F("Acc.: "));
          display.print(String(meanAccuracy)); // Call the helper function
          display.print(" -> ");
          display.print(String(desiredAccuracyInM)); // Call the helper function
          display.print(F(" m"));
          display.display();
        }
   
      }
      else {
        DEBUG_SERIAL.println(F("SVIN request failed"));
      }

      delay(1000);
    }
    DEBUG_SERIAL.println(F("Survey valid!"));
    DEBUG_SERIAL.println(F("Accuracy"));
    DEBUG_SERIAL.print(getAccuracy()); 
    DEBUG_SERIAL.println(F(" m"));
    DEBUG_SERIAL.println(F("Base survey complete! RTCM now broadcasting."));

  

    // If you were setting up a full GNSS station, you would want to save these settings.
    // Because setting an incorrect static position will disable the ability to get a lock, we will skip saving during this example
    // if (myGNSS.saveConfiguration() == false) //Save the current settings to flash and BBR
    // 

    // TODO: save location
}

void setupRTKBase(bool surveyEnabled) {
  if (myGNSS.begin(Wire) == false) {
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

  if (!surveyEnabled) {
    // Latitude, Longitude, Altitude input:
    location_int_t baseLoc;
    getIntLocationFromSPIFFS(&baseLoc, PATH_RTK_LOCATION_LATITUDE, PATH_RTK_LOCATION_LONGITUDE, PATH_RTK_LOCATION_ALTITUDE);
    response &= myGNSS.setStaticPosition(baseLoc.lat, baseLoc.lat_hp, baseLoc.lon, baseLoc.lon_hp, baseLoc.alt, baseLoc.alt_hp, true); 
    response &= myGNSS.setHighPrecisionMode(true); // TODO: test this
    // Or use Earth-centered coordinates:
    //response &= myGNSS.setStaticPosition(ECEF_X_CM, ECEF_X_HP, ECEF_Y_CM, ECEF_Y_HP, ECEF_Z_CM, ECEF_Z_HP);  //With high precision 0.1mm parts
    if (response == false) {
      DEBUG_SERIAL.println(F("Failed to enter static position. Freezing..."));
      while (1)
        ;
    } else {
      DEBUG_SERIAL.println(F("Static position set"));
    }

  } else {
    float desiredAcc = getDesiredSurveyAccuracy(PATH_RTK_LOCATION_SURVEY_ACCURACY);
    runSurvey(desiredAcc, response);
  }
  
  DEBUG_SERIAL.println(F("Module failed to save"));
  

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

void task_rtk_wifi_connection(void *pvParameters) {
    (void)pvParameters;
    Wire.setClock(I2C_FREQUENCY_100K);
    // Measure stack size
    // UBaseType_t uxHighWaterMark; 
    // uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    // DEBUG_SERIAL.print(F("task_rtk_wifi_connection setup, uxHighWaterMark: "));
    // DEBUG_SERIAL.println(uxHighWaterMark);

    bool connectionSuccess = false;
    char response[512];
    int responseSpot = 0;

    // Read credentials
    String casterHost = readFile(SPIFFS, PATH_RTK_CASTER_HOST);
    String casterPort = readFile(SPIFFS, PATH_RTK_CASTER_PORT);
    String mountPoint =  readFile(SPIFFS, PATH_RTK_MOINT_POINT);
    String mountPointPW =  readFile(SPIFFS, PATH_RTK_MOINT_POINT_PW);

    // Check RTK credentials
    bool credentialsExists = true;
    credentialsExists &= !casterHost.isEmpty();
    credentialsExists &= !casterPort.isEmpty();
    credentialsExists &= !mountPoint.isEmpty();
    credentialsExists &= !mountPointPW.isEmpty();
    
    if (!credentialsExists) {
      DEBUG_SERIAL.println("RTK Credentials incomplete, please fill out the web form and reboot!\nFreezing RTK task. ");
      while (true) {};
    }

    String locationMethod = readFile(SPIFFS, PATH_RTK_LOCATION_METHOD);
    String latitude = readFile(SPIFFS, PATH_RTK_LOCATION_LATITUDE);
    String longitude = readFile(SPIFFS, PATH_RTK_LOCATION_LONGITUDE);
    String altitude = readFile(SPIFFS, PATH_RTK_LOCATION_ALTITUDE);
    bool surveyEnabled = true;
    surveyEnabled &=  locationMethod.isEmpty() || \
                      locationMethod.equals("survey_enabled") || \
                      latitude.isEmpty() || \
                      longitude.isEmpty() || \
                      altitude.isEmpty();
    DEBUG_SERIAL.printf("task_rtk_wifi_connection, surveyEnabled: %d", surveyEnabled);
    setupRTKBase(surveyEnabled);

    while (true) {
      // beginServing() func content
      // Connect if we are not already
      taskStart:
      if (ntripCaster.connected() == false) {
          DEBUG_SERIAL.printf("Opening socket to %s\n", CASTER_HOST);

        if (ntripCaster.connect(casterHost.c_str(), (uint16_t)casterPort.toInt()) == true)  //Attempt connection
        {
            DEBUG_SERIAL.printf("Connected to %s:%d\n", casterHost.c_str(), (uint16_t)casterPort.toInt());

            const int SERVER_BUFFER_SIZE = 512;
            char serverRequest[SERVER_BUFFER_SIZE];

            snprintf(serverRequest,
                    SERVER_BUFFER_SIZE,
                    "SOURCE %s /%s\r\nSource-Agent: NTRIP SparkFun u-blox Server v1.0\r\n\r\n",
                    mountPointPW.c_str(), mountPoint.c_str());

            DEBUG_SERIAL.println(F("Sending server request:"));
            DEBUG_SERIAL.println(serverRequest);
            ntripCaster.write(serverRequest, strlen(serverRequest));

            // Wait for response
            unsigned long timeout = millis();
            while (ntripCaster.available() == 0) {
            if (millis() - timeout > 5000) {
                DEBUG_SERIAL.println(F("Caster timed out!"));
                ntripCaster.stop();
                goto taskStart;// return;
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
                DEBUG_SERIAL.print(F("Failed to connect to Caster: ")); 
                DEBUG_SERIAL.println(response);
                goto taskStart;// return;
                }
            }  // End attempt to connect
            else {
                DEBUG_SERIAL.println(F("Connection to host failed"));
                goto taskStart;// return;
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
                const String status = "RTCM timeout. Disconnecting...";
                DEBUG_SERIAL.println(status);
                ntripCaster.stop();

                if (displayConnected) {
                  display.clearDisplay();
                  display.setCursor(0,0);
                  display.print("ERROR, hang up!");
                  display.print(DEVICE_NAME);
                  display.setCursor(0,20);
                  display.print("ntripCaster stopped");
                  display.setCursor(0,30);
                  display.print(status);
                  display.display();
                }
                goto taskStart;// return;
        }

        delay(10);

        //Report some statistics every 10000
        if (millis() - lastReport_ms > 10000) {
            lastReport_ms += 10000;
            DEBUG_SERIAL.printf("kB sent: %.2f\n", serverBytesSent/1000.0);
            printHighPrecisionPositionAndAccuracy();

            double latitudeHp = getLatitudeHp();
            double longitudeHp = getLongitudeHp();
            float altitude = getHeightOverSeaLevel();
            float accuracy = getAccuracy();
            static float lastAccuracy = 100.0;
            DEBUG_SERIAL.print("lastAccuracy: "); DEBUG_SERIAL.println(lastAccuracy, 4);
            DEBUG_SERIAL.print("Accuracy: "); DEBUG_SERIAL.println(accuracy, 4);
            bool shouldUpdateDisplay = true;

            if (lastAccuracy > accuracy) {
              shouldUpdateDisplay = true;
              if (saveLocation()) {
                DEBUG_SERIAL.println(F("Location updated"));
                lastAccuracy = accuracy;
                
              } else {
                DEBUG_SERIAL.println(F("Error saving location"));
              }
            } else {
              shouldUpdateDisplay = false;
            }

            if (displayConnected && shouldUpdateDisplay) {
              display.clearDisplay();

              display.setCursor(0, 0);
              display.print(F("SSID: "));
              display.print(WiFi.SSID());

              display.setCursor(0, 10);
              display.print(F("http://"));
              display.print(DEVICE_NAME);
              display.print(F(".local"));

              display.setCursor(0, 20);
              display.print(F("Lat: "));
              display.print(latitudeHp, 9);
              display.print(F(" deg"));

              display.setCursor(0, 30);
              display.print("Lon: ");
              display.print(longitudeHp, 9);
              display.print(F(" deg"));

              display.setCursor(0, 40);
              display.print("Alt: ");
              display.print(altitude, 4);
              display.print(F(" m"));

              display.setCursor(0, 50);
              display.print(F("hAcc: "));
              display.print(accuracy, 4);
              display.print(F(" m"));
              display.display();
            }
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
 *                              Button(s)
 * ****************************************************************************/

void buttonHandler(Button2 &btn) {
  if (btn == button) {
    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_SERIAL.println(F("Wiping WiFi credentials and RTK settings from memory..."));
    wipeSpiffsFiles();
    ESP.restart();
  }
}

/*******************************************************************************
 *                                Display
 * ****************************************************************************/

bool setupDisplay() {
  displayConnected = false;
  if (!display.begin(OLED_I2C_ADDR, true)) {
    DEBUG_SERIAL.println("Could not find SH110X? Check wiring");
    // while (true) delay(100);
  } else { // Address 0x3C default
    displayConnected = true;
  }
 
  if (displayConnected) {
    display.display();
    delay(500);

    // Clear the buffer.
    display.clearDisplay();
    display.setTextSize(1);
    //display.drawLine(0, 0, display.width() - 1, 0, SH110X_WHITE);
    display.setTextColor(SH110X_WHITE, SH110X_BLACK);
    DEBUG_SERIAL.println(F("Display setup done"));
    display.setCursor(0,0);
    display.print(F("Hello from "));
    display.print(DEVICE_NAME);
    display.display();
  }

  return displayConnected;
}


String secondsToTimeFormat(uint32_t sec) {  //Time we are converting. This can be passed from another function.
  int hh = sec/3600;             //Number of seconds in an hour
  int mm = (sec-hh*3600)/60;     //Remove the number of hours and calculate the minutes.
  int ss = sec-hh*3600-mm*60;    //Remove the number of hours and minutes, leaving only seconds.
  String hhStr = hh < 10 ? ("0" + String(hh)) : String(hh);
  String mmStr = mm < 10 ? ("0" + String(mm)) : String(mm);
  String ssStr = ss < 10 ? ("0" + String(ss)) : String(ss);                          
  String hhMmmSs = (hhStr + ":" + mmStr + ":" + ssStr);

  return hhMmmSs;
}

void printHighPrecisionPositionAndAccuracy() {
    // getHighResLatitude: returns the latitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLatitudeHp: returns the high resolution component of latitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getHighResLongitude: returns the longitude from HPPOSLLH as an int32_t in degrees * 10^-7
    // getHighResLongitudeHp: returns the high resolution component of longitude from HPPOSLLH as an int8_t in degrees * 10^-9
    // getElipsoid: returns the height above ellipsoid as an int32_t in mm
    // getElipsoidHp: returns the high resolution component of the height above ellipsoid as an int8_t in mm * 10^-1
    // getMeanSeaLevel: returns the height above mean sea level as an int32_t in mm
    // getMeanSeaLevelHp: returns the high resolution component of the height above mean sea level as an int8_t in mm * 10^-1
    // getHorizontalAccuracy: returns the horizontal accuracy estimate from HPPOSLLH as an uint32_t in mm * 10^-1

    // First, let's collect the position data
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    int32_t ellipsoid = myGNSS.getElipsoid();
    int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    uint32_t accuracy = myGNSS.getHorizontalAccuracy();

    // Defines storage for the lat and lon as double
    double d_lat; // latitude
    double d_lon; // longitude

    // Assemble the high precision latitude and longitude
    d_lat = ((double)latitude) / 10000000.0;        // Convert latitude from degrees * 10^-7 to degrees
    d_lat += ((double)latitudeHp) / 1000000000.0;   // Now add the high resolution component (degrees * 10^-9 )
    d_lon = ((double)longitude) / 10000000.0;       // Convert longitude from degrees * 10^-7 to degrees
    d_lon += ((double)longitudeHp) / 1000000000.0;  // Now add the high resolution component (degrees * 10^-9 )

   // Print the lat and lon
    DEBUG_SERIAL.print("Lat (deg): ");
    DEBUG_SERIAL.print(d_lat, 9);
    DEBUG_SERIAL.print(", Lon (deg): ");
    DEBUG_SERIAL.println(d_lon, 9);

    // Now define float storage for the heights and accuracy
    float f_ellipsoid;
    float f_msl;
    float f_accuracy;

    // Calculate the height above ellipsoid in mm * 10^-1
    f_ellipsoid = (ellipsoid * 10) + ellipsoidHp;
    // Now convert to m
    f_ellipsoid = f_ellipsoid / 10000.0; // Convert from mm * 10^-1 to m

    // Calculate the height above mean sea level in mm * 10^-1
    f_msl = (msl * 10) + mslHp;
    // Now convert to m
    f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m

    // Convert the horizontal accuracy (mm * 10^-1) to a float
    f_accuracy = accuracy;
    // Now convert to m
    f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m

    // Finally, do the printing
    DEBUG_SERIAL.print(", Ellipsoid (m): ");
    DEBUG_SERIAL.print(f_ellipsoid, 4); // Print the ellipsoid with 4 decimal places

    DEBUG_SERIAL.print(", Mean Sea Level (m): ");
    DEBUG_SERIAL.print(f_msl, 4); // Print the mean sea level with 4 decimal places

    DEBUG_SERIAL.print(", Accuracy (m): ");
    DEBUG_SERIAL.println(f_accuracy, 4); // Print the accuracy with 4 decimal places
}

bool saveLocation() {
    int32_t latitude = myGNSS.getHighResLatitude();
    int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
    int32_t longitude = myGNSS.getHighResLongitude();
    int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
    // int32_t ellipsoid = myGNSS.getElipsoid();
    // int8_t ellipsoidHp = myGNSS.getElipsoidHp();
    int32_t msl = myGNSS.getMeanSeaLevel();
    int8_t mslHp = myGNSS.getMeanSeaLevelHp();
    DEBUG_SERIAL.print("saveLocation: msl: ");DEBUG_SERIAL.print(msl);
    DEBUG_SERIAL.print(", mslHp: ");DEBUG_SERIAL.println(mslHp);
    // uint32_t accuracy = myGNSS.getHorizontalAccuracy();
    bool success = true;
    String csvStr = String(latitude) + SEP + String(latitudeHp);
    success &= writeFile(SPIFFS, PATH_RTK_LOCATION_LATITUDE, csvStr.c_str());
    csvStr = "";
    csvStr = String(longitude) + SEP + String(longitudeHp);
    success &= writeFile(SPIFFS, PATH_RTK_LOCATION_LONGITUDE, csvStr.c_str());
    csvStr = "";
    csvStr = String(msl) + SEP + String(mslHp);
    success &= writeFile(SPIFFS, PATH_RTK_LOCATION_ALTITUDE, csvStr.c_str());

    return success;
}

double getLatitudeHp() {
  int32_t latitude = myGNSS.getHighResLatitude();
  int8_t latitudeHp = myGNSS.getHighResLatitudeHp();
  double d_lat; // latitude
  d_lat = ((double)latitude) / 10000000.0;        // Convert latitude from degrees * 10^-7 to degrees
  d_lat += ((double)latitudeHp) / 1000000000.0;   // Now add the high resolution component (degrees * 10^-9 )

  DEBUG_SERIAL.print("Lat (deg): ");
  DEBUG_SERIAL.println(d_lat, 9);

  return d_lat;
}

double getLongitudeHp() {
  int32_t longitude = myGNSS.getHighResLongitude();
  int8_t longitudeHp = myGNSS.getHighResLongitudeHp();
  double d_lon; // longitude
  d_lon = ((double)longitude) / 10000000.0; 
  d_lon += ((double)longitudeHp) / 1000000000.0;
  DEBUG_SERIAL.print("Lon (deg): ");
  DEBUG_SERIAL.println(d_lon, 9);

  return d_lon;
}

float getAccuracy() {
  float f_accuracy;
  uint32_t accuracy = myGNSS.getHorizontalAccuracy();
  // Convert the horizontal accuracy (mm * 10^-1) to a float
  f_accuracy = accuracy;
  // Now convert to m
  f_accuracy = f_accuracy / 10000.0; // Convert from mm * 10^-1 to m
  DEBUG_SERIAL.print("horizontal Acc.: ");
  DEBUG_SERIAL.println(f_accuracy, 4); // Print the accuracy with 4 decimal places

  return f_accuracy;
}

// double getHeightOverSeaLevelHp() {
//   int32_t msl = myGNSS.getMeanSeaLevel();
//   int8_t mslHp = myGNSS.getMeanSeaLevelHp();
//   double d_mslHp; // mean height over sea level
//   d_mslHp = ((double)msl) / 10000000.0; 
//   d_mslHp += ((double)mslHp) / 1000000000.0;
//   DEBUG_SERIAL.print("Height over sea in m: ");
//   DEBUG_SERIAL.println(d_mslHp, 9);

//   return d_mslHp;
// }

float getHeightOverSeaLevel() {
  float f_msl;
  int32_t msl = myGNSS.getMeanSeaLevel();
  int8_t mslHp = myGNSS.getMeanSeaLevelHp();
  // Calculate the height above mean sea level in mm * 10^-1
  f_msl = (msl * 10) + mslHp;
  // Now convert to m
  f_msl = f_msl / 10000.0; // Convert from mm * 10^-1 to m
  DEBUG_SERIAL.print("Alt.: ");
  DEBUG_SERIAL.println(f_msl, 4); // Print the mean sea level with 4 decimal places

  return f_msl;
}


