/********************************
 * VOC sensor parameters
 ********************************/
#define VOC_SENSOR_ID         0x5A
#define VOC_READ_CMD          0xB5


uint16_t CoreP_getCO2(void){
  uint8_t i = 0;
  uint16_t CO2_data = 0;
  uint8_t dataArray[9] = {0};

  Wire.requestFrom(VOC_SENSOR_ID, 9);
  delay(100); // Absolute maximum conversion time
  while (Wire.available())
  {
    dataArray[i] = (uint8_t)Wire.read();
    //Serial.println(dataArray[i]);
    i++;
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

  Wire.requestFrom(VOC_SENSOR_ID, 9);
  delay(100); // Absolute maximum conversion time
  while (Wire.available())
  {
    dataArray[i] = (uint8_t)Wire.read();
    //Serial.println(dataArray[i]);
    i++;
  }
  Tvoc_data = (dataArray[7] << 8 | dataArray[8]);

  if (dataArray[2] == 16){
    Tvoc_data = 0;
    Serial.println("Voc sensor still warming up");
  }
  return Tvoc_data;
}
