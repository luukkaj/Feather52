/********************************
 * VOC sensor parameters
 ********************************/
#define VOC_SENSOR_ID         0x5A
#define VOC_READ_CMD          0xB5


uint16_t CoreP_getCO2(void){
  uint8_t i = 0;
  uint16_t CO2_data = 0;
  uint8_t dataArray[9] = {0};
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    Wire.requestFrom(VOC_SENSOR_ID, 9);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in CoreP getCO2\n\n");
    return 0;
  }
  //unsigned long start_time = millis();
  //while ((millis()-start_time) < 100){} // Absolute maximum conversion time
  //delay(100);
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    uint32_t a = millis();
    while (Wire.available())
    {
      if ((millis() - a ) > 500){
        Serial.print("Wire stuck in available for 500ms in CoreP getTVOC");
      }
      dataArray[i] = (uint8_t)Wire.read();
      //Serial.println(dataArray[i]);
      i++;
    }
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in CoreP getCO2\n\n");
    return 0;
  }
  CO2_data = (dataArray[0] << 8 | dataArray[1]);

  if (dataArray[2] == 16){
    CO2_data = 0;
    Serial.println("Voc sensor still warming up");
  }
  return CO2_data;
}


uint16_t CoreP_getTVOC(void){
  uint8_t i = 0;
  uint16_t Tvoc_data = 0;
  uint8_t dataArray[9] = {0};

  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    Wire.requestFrom(VOC_SENSOR_ID, 9);
//    xSemaphoreGive(xI2CSemaphore);
//  }else{
//    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in CoreP getTVOC\n\n");
//    return 0;
//  }
//  //unsigned long start_time = millis();
//  //while ((millis()-start_time) < 100){} // Absolute maximum conversion time
    uint32_t a = millis();
//  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    while (Wire.available())
    {
      if ((millis() - a ) > 500){
        Serial.print("Wire stuck in available for 500ms in CoreP getTVOC");
      }
      dataArray[i] = (uint8_t)Wire.read();
      //Serial.println(dataArray[i]);
      i++;
    }
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in CoreP getTVOC\n\n");
    return 0;
  }
  Tvoc_data = (dataArray[7] << 8 | dataArray[8]);

  if (dataArray[2] == 16){
    Tvoc_data = 0;
    Serial.println("Voc sensor still warming up");
  }
  return Tvoc_data;
}
