#include "manager.h"
#include <PubSubClient.h>
#include <DHT.h>

#define SETUP_PIN 5

#define DHTPIN 12
#define DHTTYPE DHT22

#define MAX_ATTEMPTS_TO_CONNECT 200

#define USING_DS18B20_SENSOR true;

#ifdef USING_DS18B20_SENSOR
  #define NUM_OF_DS18B20_SENSOR 2
  
  #include <OneWire.h>
  #include <DallasTemperature.h>
  
  // Data wire is plugged into port 2 on the Arduino
  #define ONE_WIRE_BUS 12
  
  // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
  OneWire oneWire(ONE_WIRE_BUS);
  
  // Pass our oneWire reference to Dallas Temperature. 
  DallasTemperature DS18B20(&oneWire);
#endif

bool temperatureSensor = false;
bool motionSensor = false;
bool lightSensor = false;


DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient MQTT_CLIENT;


String clientName = String("esp-") + WiFi.macAddress();

Item items[] = {{"mqtt_server", "192.168.0.100"},
                {"channel", "sensor/esp"},
                {"updateFrequency", "600"}};

Item *mqtt_server = &items[0];
Item *channel = &items[1];
Item *updateFrequency = &items[2];

bool setupWiFi()
{
  int attempts = 0;

  WiFi.mode(WIFI_STA);
  
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
  float voltage = analogRead(A0) * 4.2 / 1024; //max voltage / max bits
  publishValue("voltage", String(voltage).c_str());

  if(temperatureSensor){
    Serial.println("Temp sensor gather reading");
    delay(3000);
    
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
  
    publishValue("temp", String(temp).c_str());
    publishValue("humidity", String(humidity).c_str());
  }

  #ifdef USING_DS18B20_SENSOR
    Serial.print("Requesting temperatures from DS18B20...");
    DS18B20.requestTemperatures(); // Send the command to get temperatures

    int temp = 0;
    for(int i = 0; i < NUM_OF_DS18B20_SENSOR; ++i){
      temp = DS18B20.getTempCByIndex(i);
      publishValue(String("temp_" + String(i)).c_str(), String(temp).c_str());
    }
  #endif

  if(motionSensor){
    Serial.println("Motion sensor gather reading");  
    publishValue("motion", "on");
  }

  if(lightSensor){
    int light = getLight(13);  
    Serial.println("Light reading ");
    publishValue("light", String(light).c_str());
  }
}



int getLight(int pin){
  int reading = 0;

  int maxReading = 5000;

  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delay(100);
  
  pinMode(pin, INPUT);

  while(digitalRead(pin) == LOW){
    ++reading;
    if(reading >= maxReading)
      break;
  }

  return maxReading - reading;
}


//Needs to be declared but won't be used
void loop(){
}


