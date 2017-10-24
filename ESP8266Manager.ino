#include "manager.h"
#include <PubSubClient.h>
#include <DHT.h>

#define SETUP_PIN 5

#define DHTPIN 12
#define DHTTYPE DHT22

#define MAX_ATTEMPTS_TO_CONNECT 200

DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient MQTT_CLIENT;

String clientName = String("esp-") + WiFi.macAddress();

Item items[] = {{"mqtt_server", "192.168.0.100"},
                {"channel", "sensor/esp"},
                {"updateFrequency", "60"}};

Item *mqtt_server = &items[0];
Item *channel = &items[1];
Item *updateFrequency = &items[2];

bool setupWiFi()
{
  int attempts = 0;
  
  WiFi.begin();

  //Must have a max nr of attempts or the device will be drained of battery if connectivity is poor or unavailable
  while( attempts < MAX_ATTEMPTS_TO_CONNECT ){
    Serial.print(".");
    if(WiFi.status() == WL_CONNECTED)
    {
      Serial.println("Connected to WiFi with local ip");
      return true;
    }
    ++attempts;
    delay(200);
  }

  Serial.println("WiFi connection attempts failed. Not trying anymore.");
  return false;
}

bool connectToMQTTBroker() {
  // Set our MQTT broker address and port
  MQTT_CLIENT.setServer(mqtt_server->value, 1883);
  MQTT_CLIENT.setClient(espClient);

  int attempts = 0;
  
  while (attempts < MAX_ATTEMPTS_TO_CONNECT) {
    // Attempt to connect
    Serial.println("Attempt to connect to MQTT broker");
    
    MQTT_CLIENT.connect(clientName.c_str());

    // Wait some time to space out connection requests
    delay(200);

    if(MQTT_CLIENT.connected())
    {
      Serial.println("MQTT connected");
      return true;
    }

    ++attempts;
  }

  Serial.println("Failed to connect to MQTT broker");
  return false;
}

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);

  bool enterSetupMode = digitalRead(SETUP_PIN) == HIGH;
  setupWifiManager(items, 3, enterSetupMode);  

  if(setupWiFi()){
    Serial.println(WiFi.localIP());

    connectToMQTTBroker(); 
       
    gatherReading();    
  }

  Serial.println("Entering sleep mode...");
  Serial.println(atoi(updateFrequency->value));
  ESP.deepSleep(atoi(updateFrequency->value) * 1000000);
}



String getChannelName(const char* measurementName)
{
  return String(channel->value) + String("/") + String(measurementName);
}

void publishValue(const char* valueName, const char* value)
{
  String channelName = getChannelName(valueName);
  Serial.print("Publishing ");
  Serial.print(channelName);
  Serial.print(": ");
  Serial.println(value);
  
  MQTT_CLIENT.publish(channelName.c_str(), value, true);
}

void gatherReading(){
  delay(2000);
  
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  float voltage = analogRead(A0) * 4.2 / 1024; //max voltage / max bits

  publishValue("temp", String(temp).c_str());
  publishValue("humidity", String(humidity).c_str());
  publishValue("voltage", String(voltage).c_str());
}

//Needs to be declared but won't be used
void loop(){
}


