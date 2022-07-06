#include <RTKBaseManager.h>

/********************************************************************************
*                             WiFi
* ******************************************************************************/
int RTKBaseManager::scanWiFiAPs(String* ssidBuff, int ssidBuffLen) {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

    // Delete old entries
    for (int i = 0; i < ssidBuffLen; i++) {
        ssidBuff[i] = "";
    }

    // WiFi.scanNetworks will return the number of networks found
    int n = WiFi.scanNetworks();
  
    Serial.println("scan done");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n);
        Serial.println(" network(s) found");
        for (int i = 0; i < n; i++) {
            // Print SSID and RSSI for each network found
            ssidBuff[i] = String(WiFi.SSID(i));//+ " (" + String(WiFi.RSSI(i)) + ")";
            Serial.println(ssidBuff[i]);
            delay(10);
            if (n == ssidBuffLen) break;
        }
    }
    Serial.println();
    return n;
}

bool RTKBaseManager::knownNetworkAvailable(const String& ssid, String* ssidBuff, int ssidBuffLen) {
  if (ssid.isEmpty()) return false;

  int nNetworks = scanWiFiAPs(ssidBuff, ssidBuffLen);
  for (int i=0; i<nNetworks; i++) {
    if (ssid.equals(ssidBuff[i])) {
      Serial.printf("A known network with SSID %s found, connecting...", ssidBuff[i]);
      return true;
    }
  }
  Serial.printf("No known network with SSID %s found, starting AP to enter new credentials\n", ssid);
  return false;
}

/********************************************************************************
*                             Web server
* ******************************************************************************/

void RTKBaseManager::notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void RTKBaseManager::actionRebootESP32(AsyncWebServerRequest *request) {
  Serial.println("ACTION 3!");
  request->send_P(200, "text/html", REBOOT_HTML, RTKBaseManager::processor);
  delay(3000);
  ESP.restart();
}

void RTKBaseManager::actionWipeData(AsyncWebServerRequest *request) {
  Serial.println("ACTION 2!");
  int params = request->params();
  Serial.printf("params: %d\n", params);
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("%d. POST[%s]: %s\n", i+1, p->name().c_str(), p->value().c_str());
    if (strcmp(p->name().c_str(), "wipe_button") == 0) {
      if (p->value().length() > 0) {
        Serial.printf("wipe command received: %s",p->value().c_str());
        wipeSpiffsFiles();
      } 
     }
    } 

  Serial.print(F("Data in SPIFFS was wiped out!"));
  request->send_P(200, "text/html", INDEX_HTML, processor);
}

void RTKBaseManager::actionUpdateData(AsyncWebServerRequest *request) {
  Serial.println("ACTION 1!");

  int params = request->params();
  for (int i = 0; i < params; i++) {
    AsyncWebParameter* p = request->getParam(i);
    Serial.printf("%d. POST[%s]: %s\n", i+1, p->name().c_str(), p->value().c_str());

    if (strcmp(p->name().c_str(), PARAM_WIFI_SSID) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_WIFI_SSID, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_WIFI_PASSWORD) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_WIFI_PASSWORD, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_LOCATION_METHOD) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_LOCATION_METHOD, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_LOCATION_SURVEY_ACCURACY) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_LOCATION_SURVEY_ACCURACY, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_LOCATION_LATITUDE) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_LOCATION_LATITUDE, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_LOCATION_LONGITUDE) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_LOCATION_LONGITUDE, p->value().c_str());
     } 
    }

    if (strcmp(p->name().c_str(), PARAM_RTK_LOCATION_HEIGHT) == 0) {
      if (p->value().length() > 0) {
        writeFile(SPIFFS, PATH_RTK_LOCATION_HEIGHT, p->value().c_str());
     } 
    }

  }
  Serial.println(F("Data saved to SPIFFS!"));
  request->send_P(200, "text/html", INDEX_HTML, RTKBaseManager::processor);
}

// Replaces placeholder with stored values
String RTKBaseManager::processor(const String& var) {
  if (var == PARAM_WIFI_SSID) {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    return (savedSSID.isEmpty() ? String(PARAM_WIFI_SSID) : savedSSID);
  }
  else if (var == PARAM_WIFI_PASSWORD) {
    String savedPassword = readFile(SPIFFS, PATH_WIFI_PASSWORD);
    return (savedPassword.isEmpty() ? String(PARAM_WIFI_PASSWORD) : "*******");
  }
  else if (var == PARAM_RTK_LOCATION_METHOD) {
    String savedLocationMethod = readFile(SPIFFS, PATH_RTK_LOCATION_METHOD);
    return (savedLocationMethod.isEmpty() ? String(PARAM_RTK_SURVEY_ENABLED) : savedLocationMethod);
  }
  else if (var == PARAM_RTK_LOCATION_SURVEY_ACCURACY) {
    String savedSurveyAccuracy = readFile(SPIFFS, PATH_RTK_LOCATION_SURVEY_ACCURACY);
    return (savedSurveyAccuracy.isEmpty() ? String(PARAM_RTK_LOCATION_SURVEY_ACCURACY) : savedSurveyAccuracy);
  }
  else if (var == PARAM_RTK_LOCATION_LATITUDE) {
    String savedLatitude = readFile(SPIFFS, PATH_RTK_LOCATION_LATITUDE);
    return (savedLatitude.isEmpty() ? String(PARAM_RTK_LOCATION_LATITUDE) : savedLatitude);
  }
  else if (var == PARAM_RTK_LOCATION_LONGITUDE) {
    String savedLongitude = readFile(SPIFFS, PATH_RTK_LOCATION_LONGITUDE);
    return (savedLongitude.isEmpty() ? String(PARAM_RTK_LOCATION_LONGITUDE) : savedLongitude);
  }
  else if (var == PARAM_RTK_LOCATION_HEIGHT) {
    String savedHeight = readFile(SPIFFS, PATH_RTK_LOCATION_HEIGHT);
    return (savedHeight.isEmpty() ? String(PARAM_RTK_LOCATION_HEIGHT) : savedHeight);
  }
  else if (var == "next_addr") {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    String savedPW = readFile(SPIFFS, PATH_WIFI_PASSWORD);
    if (savedSSID.isEmpty() || savedPW.isEmpty()) {
      return String(IP_AP);
    } else {
      String clientAddr = String(DEVICE_NAME);
      clientAddr += ".local";
      return clientAddr;
    }
  }
  else if (var == "next_ssid") {
    String savedSSID = readFile(SPIFFS, PATH_WIFI_SSID);
    return (savedSSID.isEmpty() ? String(SSID_AP) : savedSSID);
  }
  return String();
}

/********************************************************************************
*                             SPIFFS
* ******************************************************************************/

String RTKBaseManager::readFile(fs::FS &fs, const char* path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");

  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;

  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);

  return fileContent;
}

void RTKBaseManager::writeFile(fs::FS &fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
void RTKBaseManager::listFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();
 
  while (file) {
      Serial.print("FILE: ");
      Serial.println(file.name());
      file = root.openNextFile();
  }
  file.close();
  root.close();
}

void RTKBaseManager::wipeSpiffsFiles() {
  File root = SPIFFS.open("/");
  File file = root.openNextFile();

  Serial.println(F("Wiping: "));

  while (file) {
    Serial.print("FILE: ");
    Serial.println(file.path());
    SPIFFS.remove(file.path());
    file = root.openNextFile();
  }
}