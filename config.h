#define FILE_PATH "/config6.json"

void loadConfig(struct Item items[], size_t itemsLength) {
  
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists(FILE_PATH)) {
      //file exists, reading and loading
      Serial.println("reading config file");
      
      File configFile = SPIFFS.open(FILE_PATH, "r");
      
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        
        json.printTo(Serial);

        Serial.println(items[0].name);
        Serial.println(items[1].name);
        Serial.println(items[2].name);
        
        if (json.success()) {
          Serial.println("\nparsed json");

          for(int i = 0 ; i < itemsLength ; ++i){
            Serial.println(items[i].name);

            Serial.println((const char*)json[items[i].name]);

            strcpy(items[i].value, json[items[i].name]);
            Serial.print("copied ");
            Serial.print(i);
            Serial.print(" - ");
            Serial.println(items[i].value);
          }

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
}
