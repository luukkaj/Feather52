#define TRIES 8

/* Function tries to read the sensor value TRIES times and returns it as a float.
 *  Returns NULL if:
 *  - Relevant sensor is not defined
 *  - Sensor read fails TRIES-times
 *  - DHT sensor gives a value <0 or >40
 */
float getTemperature(){
  float BLE_temperature = NULL;
  uint8_t nr_of_tries = 0;
  while ((BLE_temperature == NULL) && (nr_of_tries < TRIES)){
    #if BAROMETER_BMP280
      BLE_temperature = bmp280.getTemperature();
    #else if TEMPHUM_DHT
      BLE_temperature = dht.readTemperature();
    #endif
    
    if (!(BLE_temperature > 0) && (!(BLE_temperature < 40))){
      BLE_temperature = NULL;
//      unsigned long start_time = millis();
//      while ((millis()-start_time) < 10){}
        delay(20);
    }
    
    nr_of_tries++;
    if (nr_of_tries == TRIES){
      Serial.println("Failed to read Temperature\n-------------------------------");
      BLE_temperature = 0;
    }
  }
  return BLE_temperature;
}

/* Function tries to read the sensor value TRIES times and returns it as a float.
 *  Returns NULL if:
 *  - Relevant sensor is not defined
 *  - Sensor read fails TRIES-times
 */
float getHumidity(){
  float BLE_humidity = 0;
  uint8_t nr_of_tries = 0;
  
  while ((BLE_humidity == 0) && (nr_of_tries < TRIES)){
    #ifdef TEMPHUM_DHT
      BLE_humidity = dht.readHumidity();
      if ((!BLE_humidity) || (BLE_humidity>=100) || (BLE_humidity <= 0)) {
        BLE_humidity = 0;
      unsigned long start_time = millis();
      while ((millis()-start_time) < 10){}
        //delay(20);
        Serial.print("-*");
      }
    #endif
    nr_of_tries++;
    if (nr_of_tries == TRIES){
      Serial.println("Failed to read Humidity\n-------------------------------------------");
      BLE_humidity = 0;
    }
  }
  return BLE_humidity;
}



float getPressure(){
  float BLE_pressure = NULL;
  uint8_t nr_of_tries = 0;
  while ((BLE_pressure == NULL) && (nr_of_tries < TRIES)){
    #ifdef TEMPHUM_DHT
      BLE_pressure = bmp280.getPressure();
      if (!BLE_pressure){
        BLE_pressure = NULL;
      }
    #endif
    nr_of_tries++;
    if (nr_of_tries == TRIES){
      Serial.println("Failed to read Pressure\n---------------------------------------");
      BLE_pressure = 0;
    }
  }
  
  return BLE_pressure;
}

