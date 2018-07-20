#define TRIES 3

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
    #if SHT31
      BLE_temperature = SHT31_getTemperature();
    #elif BAROMETER_BMP280
      BLE_temperature = bmp280.getTemperature();
    #elif TEMPHUM_DHT
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
uint32_t lastReadHumidityTime = 0;
float lastHumidityValue = 0;

float getHumidity(){
  
  float BLE_humidity = 0;
  uint8_t nr_of_tries = 0;
  
  if (((millis() - lastReadHumidityTime) < (1 * 1000)) && (lastHumidityValue > 0)){
    return lastHumidityValue;
  }else{
    while ((BLE_humidity == 0) && (nr_of_tries < TRIES)){
      #if SHT31
        BLE_humidity = SHT31_getHumidity();
      #elif TEMPHUM_DHT
        BLE_humidity = dht.readHumidity();
        if ((!BLE_humidity) || (BLE_humidity>=100) || (BLE_humidity <= 0)) {
          BLE_humidity = 0;
        //unsigned long start_time = millis();
        //while ((millis()-start_time) < 10){}
          delay(20);
          Serial.print("-*");
        }
      #endif
      nr_of_tries++;
      if (nr_of_tries == TRIES){
        Serial.println("Out of tries to read Humidity\n-------------------------------------------");
        BLE_humidity = 0;
      }
    }
    lastHumidityValue = BLE_humidity;
    lastReadHumidityTime = millis();
    return BLE_humidity;
  }
}



float getPressure(){
  float BLE_pressure = NULL;
  uint8_t nr_of_tries = 0;
  while ((BLE_pressure == NULL) && (nr_of_tries < TRIES)){
    #ifdef TEMPHUM_DHT
      
      if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
        BLE_pressure = bmp280.getPressure();
        xSemaphoreGive(xI2CSemaphore);
      }else{
        Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in getPressure\n\n");
      }
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

float getDewPoint(){//float lastDewPoint){
  float RH = getHumidity();
  float T = getTemperature();

  float dew_point = 243.04*(log(RH/100.0)+((17.625*T)/(243.04+T)))/(17.625-log(RH/100.0)-((17.625*T)/(243.04+T)));
  return dew_point;
}


