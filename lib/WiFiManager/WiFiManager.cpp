#include "WiFiManager.h"

WebServer server(80);

void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

bool writeToMemory(String ssid, String pass) {
  unsigned int bufLen = 30;
  char ssidBuf[bufLen];
  char passBuf[bufLen];
  ssid.toCharArray(ssidBuf, bufLen);
  pass.toCharArray(passBuf, bufLen); 
  EEPROM.writeString(SSID_ADDR, ssidBuf);
  EEPROM.writeString(KEY_ADDR, passBuf);
  delay(100);
  String s = EEPROM.readString(SSID_ADDR);
  String p = EEPROM.readString(KEY_ADDR);
  //#if DEBUG
  DEBUG_SERIAL.println(F("Stored SSID and KEY are: "));
  DEBUG_SERIAL.print(F("SSID: "));
  DEBUG_SERIAL.println(s);
  DEBUG_SERIAL.print(F("KEY: "));
  DEBUG_SERIAL.println(p);
  //#endif
  if (ssid == s && pass == p) {
    return true;  
  } else {
    return false;
  }
}

void handleSubmit() {
  String response_success="<h1>Success</h1>";
  response_success +="<h2>Device will restart in 3 seconds</h2>";

  String response_error="<h1>Error</h1>";
  response_error +="<h2><a href='/'>Go back</a>to try again";
  
  if (writeToMemory(String(server.arg("ssid")),String(server.arg("password")))) {
     server.send(200, "text/html", response_success);
     EEPROM.commit();
     delay(3000);
     ESP.restart();
  } else {
     server.send(200, "text/html", response_error);
  }
}

void handleRoot() {
  if (server.hasArg("ssid")&& server.hasArg("password")) {
    handleSubmit();
  }
  else {
    server.send(200, "text/html", INDEX_HTML);
  }
}

bool loadWiFiCredsForm() {
  String s = EEPROM.readString(SSID_ADDR);
  String p = EEPROM.readString(KEY_ADDR);
  
  Serial.println(F("Setting Access Point..."));
  const String ssidAP = getDeviceName(DEVICE_TYPE_PREFIX);
  DEBUG_SERIAL.print("ssidAP: ");
  DEBUG_SERIAL.println(ssidAP);
  WiFi.softAP(ssidAP.c_str(), DEFAULT_KEY);
  
  IPAddress IP = WiFi.softAPIP();
  
  DEBUG_SERIAL.print(F("AP IP address: "));
  DEBUG_SERIAL.println(IP);
  
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();
  
  DEBUG_SERIAL.println(F("HTTP server started"));
 
  while (s.length() <= 0 || p.length() <= 0) {
    server.handleClient();
    delay(3000);
    DEBUG_SERIAL.print(".");
  }
  
  return false;
}

bool checkWiFiCreds() {
  DEBUG_SERIAL.println(F("Checking WIFI credentials"));
  String s = EEPROM.readString(SSID_ADDR);
  String p = EEPROM.readString(KEY_ADDR);
  //#if DEBUG
  DEBUG_SERIAL.println(F("Found credentials in EEPROM!"));
  DEBUG_SERIAL.print(F("SSID: "));
  DEBUG_SERIAL.println(s);
  DEBUG_SERIAL.print(F("KEY: "));
  DEBUG_SERIAL.println(p);
  delay(5000);
  //#endif
  if (s.length() > 0 && p.length() > 0) {
    return true;
  } else {
    return false;
  }
}

void wipeEEPROM() {
  for(int i=0; i<END_ADDR; i++){
    EEPROM.writeByte(i,0x00);
  }
  EEPROM.commit();
}