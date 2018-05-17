#include "manager.h"
#include <PubSubClient.h>


#define SETUP_PIN 13 //Used to be 5
#define SENSOR_POWER_PIN 14

#define MAX_ATTEMPTS_TO_CONNECT 200

//#define USING_DS18B20_SENSOR false;
//#define USING_DHT22_SENSOR true;
//#define USING_BMP180_SENSOR true;
#define USING_BMP280_SENSOR true;
//#define USING_AM2320_SENSOR true;
//#define USING_TSL2561_LIGHT_SENSOR true;

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

#ifdef USING_DHT22_SENSOR
  #include <DHT.h>

  #define DHTPIN 14 //Used to be 12
  #define DHTTYPE DHT22

  DHT dht(DHTPIN, DHTTYPE);
#endif

#ifdef USING_BMP180_SENSOR
  #include <Adafruit_BMP085.h>
  
  Adafruit_BMP085 bmp;
#endif

#ifdef USING_BMP280_SENSOR
  #include <Adafruit_BMP280.h>
  
  Adafruit_BMP280 bmp;
#endif

#ifdef USING_AM2320_SENSOR
  #include <AM2320.h>

  AM2320 am2320;
#endif

#ifdef USING_TSL2561_LIGHT_SENSOR
  #include <Adafruit_Sensor.h>
  #include <Adafruit_TSL2561_U.h>
  Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
#endif

bool motionSensor = false;
bool lightSensor = false;



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
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH);

  bool enterSetupMode = digitalRead(SETUP_PIN) == HIGH;
  setupWifiManager(items, 3, enterSetupMode);  

  if(setupWiFi()){
    Serial.println(WiFi.localIP());

    connectToMQTTBroker(); 
       
    gatherReading();    
  }

  digitalWrite(SENSOR_POWER_PIN, LOW);
  
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
  #ifdef USING_DHT22_SENSOR
    Serial.println("Temp sensor gather reading");
    delay(3000);
    
    float temp = dht.readTemperature();
    float humidity = dht.readHumidity();
  
    publishValue("temp", String(temp).c_str());
    publishValue("humidity", String(humidity).c_str());
  #endif

  #ifdef USING_AM2320_SENSOR
    Serial.println("AM2320 ready");
    am2320.begin();

    if (am2320.measure()) {
      publishValue("temp", String(am2320.getTemperature()).c_str());
      publishValue("humidity", String(am2320.getHumidity()).c_str());
    }
    else { 
      int errorCode = am2320.getErrorCode();
      switch (errorCode) {
        case 1: Serial.println("ERR: AM2320 Sensor is offline"); break;
        case 2: Serial.println("ERR: AM2320 CRC validation failed."); break;
      }    
    }
  #endif

  #ifdef USING_DS18B20_SENSOR
    Serial.println("Requesting temperatures from DS18B20...");
    DS18B20.requestTemperatures(); // Send the command to get temperatures

    float ds18b20_temp = 0;
    for(int i = 0; i < NUM_OF_DS18B20_SENSOR; ++i){
      ds18b20_temp = DS18B20.getTempCByIndex(i);
      publishValue(String("temp_" + String(i)).c_str(), String(ds18b20_temp).c_str());
    }
  #endif

  #ifdef USING_BMP180_SENSOR
    if (bmp.begin()) {
      Serial.println("BMP180 ready");
      publishValue("pressure", String(bmp.readPressure() / 100.0).c_str());
      publishValue("temp2", String(bmp.readTemperature()).c_str());
    }
    else { 
      Serial.println("BMP180 sensor not found");
    }
  #endif

    #ifdef USING_BMP280_SENSOR
    if (bmp.begin()) {
      Serial.println("BMP280 ready");
      publishValue("pressure", String(bmp.readPressure() / 100.0).c_str());
      publishValue("temp2", String(bmp.readTemperature()).c_str());
    }
    else { 
      Serial.println("BMP280 sensor not found");
    }
  #endif

  #ifdef USING_TSL2561_LIGHT_SENSOR
    if(!tsl.begin()){
      Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");  
    }
    else{
      Serial.println("TSL2561 ready");
    }

    sensors_event_t tslEvent;
    tsl.getEvent(&tslEvent);
    unsigned int luminosity = 0;
    if (tslEvent.light)
    {
      luminosity = tslEvent.light;  
    }
    
    publishValue("light", String(luminosity).c_str());

  #endif

  if(motionSensor){
    Serial.println("Motion sensor gather reading");  
    publishValue("motion", "on");
  }

  float voltage = analogRead(A0) * 4.2 / 1024; //max voltage / max bits
  publishValue("voltage", String(voltage).c_str());

  long rssi = WiFi.RSSI();
  publishValue("rssi", String(rssi).c_str());
}

//Needs to be declared but won't be used
void loop(){
}


