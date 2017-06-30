#include "manager.h"

#include <DHT.h>

#define SETUP_PIN 5

#define DHTPIN 12
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
WiFiClient client;

Item items[] = {{"server", "api.thingspeak.com"},
                {"apiKey", "APIKEY"},
                {"updateFrequency", "60"}};

Item *server = &items[0];
Item *apiKey = &items[1];
Item *updateFrequency = &items[2];
      
void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);

  bool enterSetupMode = digitalRead(SETUP_PIN) == HIGH;
  setupWifiManager(items, 3, enterSetupMode);  

  int attempts = 0;
  
  WiFi.begin();

  //Must have a max nr of attempts or the device will be drained of battery if connectivity is poor or unavailable
  while( attempts < 200 ){
    Serial.print(".");
    if(WiFi.status() == WL_CONNECTED){
      break;
    }
    ++attempts;
    delay(200);
  }

  if(attempts >= 200){
    Serial.println("Connection attempts failed. Not trying anymore.");
  }
  else{
    Serial.println("local ip");
    Serial.println(WiFi.localIP());
    gatherReading();    
  }

  Serial.println("Entering sleep mode...");
  Serial.println(atoi(updateFrequency->value));
  ESP.deepSleep(atoi(updateFrequency->value) * 1000000);
}

void gatherReading(){
//  float voltage = analogRead(A0) * 4.2 / 1024; //max voltage / max bits
////  int light = getLight(13);
//  
//  delay(5000);//Needed to measure correctly
//
//  float temp = dht.readTemperature();
//  float humidity = dht.readHumidity();
//  
//  if( client.connect( items[0].value, 80 ) ) {
//
//    String postStr = items[1].value; //API Key
//    postStr +="&field1=";
//    postStr += String(temp);
//    postStr +="&field2=";
//    postStr += String(humidity);
//    postStr +="&field3=";
//    postStr += String(voltage);
////    postStr +="&field4=";
////    postStr += String(light);
//
//    client.print("POST /update HTTP/1.1\n");
//
//    char strBuffer[100];
//    strcat(strBuffer, "Host: ");
//    strcat(strBuffer, items[0].value);
//    strcat(strBuffer, "\n");
//    client.print(strBuffer);
//    
//    client.print("Connection: close\n");
//
//    strcpy(strBuffer, "X-THINGSPEAKAPIKEY: ");
//    strcat(strBuffer, items[1].value); //API key;
//    strcat(strBuffer, "\n");
//    
//    client.print(strBuffer);
//    client.print("Content-Type: application/x-www-form-urlencoded\n");
//    client.print("Content-Length: ");
//    client.print(postStr.length());
//    client.print("\n\n");
//    client.print(postStr);
//    
//    Serial.print(postStr);
//    delay(50);
//  }
//  
//  client.stop();
}

//Needs to be declared but won't be used
void loop(){
}


