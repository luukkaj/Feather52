#ifdef ENABLE_BLUETOOTH


/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <bluefruit.h>
#include "customGATT.h"
#include "semphr.h"

/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37
 * Body Sensor Location Char:   0x2A38
 */
//BLEService        environmental_sensing_service   = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
//BLECharacteristic temperature_characteristic      = BLECharacteristic(UUID16_CHR_TEMPERATURE);
//BLECharacteristic humidity_characteristic         = BLECharacteristic(UUID16_CHR_HUMIDITY);
//BLECharacteristic pressure_characteristic         = BLECharacteristic(UUID16_CHR_PRESSURE);


BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance


bool writeTemperature(float temp){
  if (temperature_characteristic.write32((int16_t)(temp*100))){ // write32 for int
    Serial.print("Wrote temperature:\t");Serial.println(temp);
    return true;
  }else{
    return false;
  }
}

bool writeHumidity(float humidity){
  if (humidity_characteristic.write16((uint16_t)(humidity*100))){
    Serial.print("Wrote humidity:\t\t");Serial.println(humidity);
    return true;
  }else{
    return false;
  }
}
bool writePressure(float pressure){
  if (pressure_characteristic.write32((uint32_t)(pressure*10))){
    Serial.print("Wrote pressure:\t\t");Serial.println(pressure);
    return true;
  }else{
    return false;
  }
}

bool writeTVOC(uint16_t tvoc){
  if (tvoc_characteristic.write16((uint16_t)(tvoc))){
    Serial.print("Wrote tvoc:\t\t");Serial.println((uint16_t)tvoc);
    return true;
  }else{
    return false;
  }
}

bool writeDewPoint(float dew){
  if (dew_point_characteristic.write8((int8_t)(dew))){
    Serial.print("Wrote dew point:\t");Serial.println((int8_t)dew);
    return true;
  }else{
    return false;
  }
}
bool writeCoolerTemperature(float temp){ 
  if (cooler_temp_characteristic.write32((int16_t)(temp*100))){ // write32 for int
    Serial.print("Wrote cooler temperature:\t");Serial.println(temp);
    return true;
  }else{
    return false;
  }
}
bool writeTension(uint16_t tension){
  if (tension_characteristic.write16((uint16_t)(tension))){
    Serial.print("Wrote watertension:\t");Serial.println((uint16_t)tension);
    return true;
  }else{
    return false;
  }
}





bool notifyTemperature(float temp){
  if (temperature_characteristic.notify32((int16_t)(100*temp))){// write32 for int
    Serial.print("Notifyed temperature:\t");Serial.println(temp);
    return true;
  }else{
    return false;
  }
}
bool notifyHumidity(float humidity){
  if (humidity_characteristic.notify16((uint16_t)(100*humidity))){
    Serial.print("Notifyed humidity:\t");Serial.println(humidity);
    return true;
  }else{
    return false;
  }
}
bool notifyPressure(float pressure){
  if (pressure_characteristic.notify32((uint32_t)(pressure*10))){
    Serial.print("Notifyed pressure:\t");Serial.println(pressure);
    return true;
  }else{
    return false;
  }
}
bool notifyDewPoint(float dew){
  if (dew_point_characteristic.notify8((int8_t)(dew))){
    Serial.print("Notifyed dew point:\t");Serial.println(dew);
    return true;
  }else{
    return false;
  }
}
bool notifyCoolerTemperature(float temp){ 
  if (cooler_temp_characteristic.notify32((int16_t)(temp*100))){ // write32 for int
    Serial.print("Notifyed cooler temperature:\t");Serial.println(temp);
    return true;
  }else{
    return false;
  }
}


void setupBluetooth()
{
  //SemaphoreHandle_t xBLEWriteSemaphore = xSemaphoreCreateBinary();
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin();
  // Set power to max
  Bluefruit.setTxPower(4);

  // Set the advertised device name (keep it short!)
  Serial.println("Setting Device Name");
  Bluefruit.setName("IAQ Feather52");

  // Set the connect/disconnect callback handlers
  Bluefruit.setConnectCallback(connect_callback);
  Bluefruit.setDisconnectCallback(disconnect_callback);

  // Configure and Start the Device Information Service
  Serial.println("Configuring the Device Information Service");
  bledis.setManufacturer("Aalto University ELEC");
  bledis.setModel("Feather nRF52");
  bledis.begin();

  // Start the BLE Battery Service and set it to 100%
  Serial.println("Configuring the Battery Service");
  blebas.begin();
  blebas.write(100);

  // Setup the Heart Rate Monitor service using
  // BLEService and BLECharacteristic classes
  Serial.println("Configuring the Temp Monitor Service");
  setupBLE_EnvironmentService();

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising payload(s)");
  startAdv();

  Serial.println("Ready Player One!!!");
  Serial.println("\nAdvertising");
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include Environmental Service UUID
  Bluefruit.Advertising.addService(environmental_sensing_service);

  // Include Name
  Bluefruit.Advertising.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms 244= 152,5ms 668=417,5ms
  Bluefruit.Advertising.setFastTimeout(5);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

void setupBLE_EnvironmentService(void)
{
  // Configure the Environmental Sensing service
  // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.environmental_sensing.xml
  // Supported Characteristics:
  // Name                         UUID    Requirement Properties
  // ---------------------------- ------  ----------- ----------
  // Heart Rate Measurement       0x2A37  Mandatory   Notify
  // Body Sensor Location         0x2A38  Optional    Read
  // Heart Rate Control Point     0x2A39  Conditional Write       <-- Not used here
  environmental_sensing_service.begin();



#if TEMPERATURE
  temperature_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    #if BAROMETER_BMP280
      temperature_characteristic.setUserDescriptor("BMP280 Temp");
    #elif SHT31
      temperature_characteristic.setUserDescriptor("Sensirion SHT31 Temp");
    #else
      temperature_characteristic.setUserDescriptor("DHT Temp");
    #endif
  temperature_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  temperature_characteristic.setFixedLen(2);
  temperature_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  temperature_characteristic.begin();
  #endif

  #if HUMIDITY
  humidity_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  #if SHT31
    humidity_characteristic.setUserDescriptor("Sensirion SHT31 Humidity");
  #else
    humidity_characteristic.setUserDescriptor("DHT Humidity");
  #endif
  humidity_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humidity_characteristic.setFixedLen(2);
  humidity_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  humidity_characteristic.begin();
  #endif

  #if PRESSURE
  pressure_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  pressure_characteristic.setUserDescriptor("BMP280 Pressure");
  pressure_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pressure_characteristic.setFixedLen(4);
  pressure_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  pressure_characteristic.begin();
  #endif

  #if VOC
  tvoc_characteristic.setProperties(CHR_PROPS_READ); // | CHR_PROPS_NOTIFY);
  tvoc_characteristic.setUserDescriptor("TVOC");
  tvoc_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tvoc_characteristic.setFixedLen(2);
  //tvoc_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  tvoc_characteristic.begin();
  #endif

  #if DEW_POINT
  dew_point_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  dew_point_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  dew_point_characteristic.setFixedLen(1);
  dew_point_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  dew_point_characteristic.begin(); 
  #endif

  #if COOLER_TEMP
  cooler_temp_characteristic.setProperties(CHR_PROPS_READ);// | CHR_PROPS_NOTIFY);
  cooler_temp_characteristic.setUserDescriptor("Condensser temperature");
  cooler_temp_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cooler_temp_characteristic.setFixedLen(2);
  //cooler_temp_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  cooler_temp_characteristic.begin();
  #endif

  #if TENSION
  tension_characteristic.setProperties(CHR_PROPS_READ);// | CHR_PROPS_NOTIFY);
  tension_characteristic.setUserDescriptor("Water tension measurement");
  tension_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  tension_characteristic.setFixedLen(2);
  tension_characteristic.begin();
  #endif
  
}

void connect_callback(uint16_t conn_handle)
{
  //xSemaphoreGive( xBLEWriteSemaphore );
  
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
  xSemaphoreGive(xBLEWriteSemaphore);
  //vTaskResume(TaskHandle_Write);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");


}

//void read_callback(BLECharacteristic& chr, ble_gatts_evt_read_t * request){
//  Serial.println("\n****************\nRead Callback\n");
//  if (chr.uuid == temperature_characteristic.uuid){
//    Serial.println("UUID matches temperature uuid");
//  }else if (chr.uuid == humidity_characteristic.uuid){
//    Serial.println("UUID matches humidity uuid");
//  }
//}


void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.println(cccd_value);
    Serial.println("");
    if (cccd_value != 0){
      xSemaphoreGive(xBLENotifySemaphore);
      //vTaskResume(TaskHandle_Notify);
    }

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr.uuid == temperature_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Temperature Measurement 'Notify' enabled");
        } else {
            Serial.println("Temperature Measurement 'Notify' disabled");
        }
    }
    else if (chr.uuid == humidity_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Humidity Measurement 'Notify' enabled");
        } else {
            Serial.println("Humidity Measurement 'Notify' disabled");
        }
    }
    else if (chr.uuid == pressure_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Pressure Measurement 'Notify' enabled");
        } else {
            Serial.println("Pressure Measurement 'Notify' disabled");
        }
    }
    else if (chr.uuid == tvoc_characteristic.uuid) {
      if (chr.notifyEnabled()) {
          Serial.println("TVOC Measurement 'Notify' enabled");
      } else {
          Serial.println("TVOC Measurement 'Notify' disabled");
      }
    }
    else if (chr.uuid == dew_point_characteristic.uuid) {
      if (chr.notifyEnabled()) {
          Serial.println("Dew Point Measurement 'Notify' enabled");
      } else {
          Serial.println("Dew Point Measurement 'Notify' disabled");
      }
    }
    else if (chr.uuid == cooler_temp_characteristic.uuid) {
      if (chr.notifyEnabled()) {
          Serial.println("Cooler Temp Measurement 'Notify' enabled");
      } else {
          Serial.println("Cooler Temp Measurement 'Notify' disabled");
      }
    }
}

#endif
