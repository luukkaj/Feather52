#if SHT31
/********************************
 * VOC sensor parameters
 ********************************/
#define SHT31_ID          0x44
#define FAST_STRETCH      

bool sht31Setup(){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    //SHT31_writeRegister(0x2C, 0x06); // High repeatability with clockstreching
    //SHT31_writeRegister(0x2C, 0x10); // Low repeatability with streching
      SHT31_writeRegister(0x24, 0x06); // High repeatability without streching
    //SHT31_writeRegister(0x24, 0x16); // Low repeatability without streching
  xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in SHT31 Setup\n\n");
  }
}

uint32_t SHT31_getMeasurement(void){
  uint32_t Temp   = 0;
  uint16_t Hum    = 0;
  uint8_t TempCRC = 0;
  uint8_t HumCRC  = 0;

  
  

  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    SHT31_writeRegister(0x2C, 0x06);
    Wire.requestFrom(SHT31_ID, 6);

    uint32_t atime = millis();
    while(Wire.available()<6){if (millis()-atime > 250){ Serial.println("\n***********************\nSHT31 timed out\n");}}
    Temp     = (Wire.read() << 8);
    Temp    |= Wire.read();
    TempCRC  = Wire.read();
    Hum      = (Wire.read() << 8);
    Hum     |= Wire.read();
    HumCRC   = Wire.read();
    //Serial.print("Temp: ");Serial.println(Temp);
    //Serial.print("Hum:  ");Serial.println(Hum);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in SHT31 get Measurement\n\n");
  }
  return ((Temp << 16) | Hum);
}

float SHT31_getTemperature(void){
  uint32_t measurement = SHT31_getMeasurement();
  uint16_t temp = (measurement >> 16);
  //Serial.print(" -temp: ");Serial.println(temp);
  //Serial.print(" -meas: ");Serial.println(measurement);
  return (float)(-45+175*(((float)temp)/(65536-1)));
}

float SHT31_getHumidity(void){
  uint32_t measurement = SHT31_getMeasurement();
  uint16_t humidity = (measurement | 0xFF);
  //Serial.print(" -hum:  ");Serial.println(humidity);
  //Serial.print(" -meas: ");Serial.println(measurement);
  return (float)(100*(((float)humidity)/(65536-1)));
}

void SHT31_writeRegister(uint8_t reg, uint8_t val)
{
//  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    Wire.beginTransmission(SHT31_ID); // start transmission to device
    Wire.write(reg);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
//  xSemaphoreGive(xI2CSemaphore);
//  }else{
//    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in SHT31 write register \n\n");
//  }
}

#endif
