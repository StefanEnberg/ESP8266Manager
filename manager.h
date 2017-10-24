#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#define MAX_ITEM_LENGTH 50

#define SETUP_WIFI_SSID "MooshiiWooshiiSensor"
#define SETUP_WIFI_PASSWORD "fooreal0"

struct Item {
  char name[MAX_ITEM_LENGTH];
  char value[MAX_ITEM_LENGTH]; 
};

#include "config.h"

//flag for saving data
bool shouldSaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setupWifiManager(struct Item items[], size_t itemsLength, bool enterSetupMode = false){
  loadConfig(items, itemsLength);

  if(enterSetupMode){
    WiFiManagerParameter * params[itemsLength];
    
    for(int i = 0 ; i < itemsLength ; i++){
      params[i] = new WiFiManagerParameter(items[i].name, items[i].name, items[i].value, MAX_ITEM_LENGTH);
    }
    
    WiFiManager wifiManager;
  
    wifiManager.resetSettings();
      
    //set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);
  
    //set static ip
    wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
    for(int i = 0 ; i < itemsLength ; i++){
      wifiManager.addParameter(params[i]);
    }
  
    if (!wifiManager.autoConnect(SETUP_WIFI_SSID, SETUP_WIFI_PASSWORD)) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      delay(5000);
    }
  
    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
  
    //save the custom parameters to FS
    if (shouldSaveConfig) {
      Serial.println("saving config");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& json = jsonBuffer.createObject();
  
      for(int i = 0 ; i < itemsLength ; i++){
        json[params[i]->getID()] = params[i]->getValue();
      }
  
      File configFile = SPIFFS.open(FILE_PATH, "w");
      if (!configFile) {
        Serial.println("failed to open config file for writing");
      }
  
      json.printTo(Serial);
      json.printTo(configFile);
      configFile.close();
    }
  }
}
