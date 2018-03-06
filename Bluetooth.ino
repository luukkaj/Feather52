#ifdef ENABLE_BLUETOOTH
#define UUID16_SVC_ENVIRONMENTAL_SENSING  0x181A
#define UUID16_CHR_TEMPERATURE            0x2A6E
#define UUID16_CHR_HUMIDITY               0x2A6F
#define UUID16_CHR_PRESSURE               0x2A6D

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

/* HRM Service Definitions
 * Heart Rate Monitor Service:  0x180D
 * Heart Rate Measurement Char: 0x2A37
 * Body Sensor Location Char:   0x2A38
 */
BLEService        environmental_sensing_service   = BLEService(UUID16_SVC_ENVIRONMENTAL_SENSING);
BLECharacteristic temperature_characteristic      = BLECharacteristic(UUID16_CHR_TEMPERATURE);
BLECharacteristic humidity_characteristic         = BLECharacteristic(UUID16_CHR_HUMIDITY);
BLECharacteristic pressure_characteristic         = BLECharacteristic(UUID16_CHR_PRESSURE);


BLEDis bledis;    // DIS (Device Information Service) helper class instance
BLEBas blebas;    // BAS (Battery Service) helper class instance


bool writeTemperature(float temp){
  if (temperature_characteristic.write16((uint16_t)temp)){
      return true;
    }else{
      return false;
    }
}
bool writeHumidity(float humidity){
  if (humidity_characteristic.write16((uint16_t)humidity)){
      return true;
    }else{
      return false;
    }
}


bool notifyTemperature(int16_t temp){
  bool validation = temperature_characteristic.notify32(temp);
  return validation;
}
bool notifyHumidity(int16_t hum){
  bool validation = humidity_characteristic.notify32(hum);
  return validation;
}
bool notifyPressure(uint32_t pressure){
  bool validation = pressure_characteristic.notify32(pressure);
  return validation;
}


void setupBluetooth()
{


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
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(60);      // number of seconds in fast mode
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

  temperature_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  #ifdef BAROMETER_BMP280
    temperature_characteristic.setUserDescriptor("BMP280 Temp");
  #else
    temperature_characteristic.setUserDescriptor("DHT Temp");
  #endif
  temperature_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  temperature_characteristic.setFixedLen(2);
  temperature_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  //temperature_characteristic.setReadAuthorizeCallback(read_callback);  // Optionally capture CCCD updates
  temperature_characteristic.begin();
  //uint8_t hrmdata[2] = { 0b00000110, 0x40 }; // Set the characteristic to use 8-bit values, with the sensor connected and detected
  //temperature_characteristic.notify(hrmdata, 2);                   // Use .notify instead of .write!
  //temperature_characteristic.notify();

  humidity_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  humidity_characteristic.setUserDescriptor("DHT Humidity");
  humidity_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  humidity_characteristic.setFixedLen(2);
  humidity_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  humidity_characteristic.begin();

  pressure_characteristic.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  pressure_characteristic.setUserDescriptor("BMP280 Pressure");
  pressure_characteristic.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  pressure_characteristic.setFixedLen(4);
  pressure_characteristic.setCccdWriteCallback(cccd_callback);  // Optionally capture CCCD updates
  pressure_characteristic.begin();


  
}

void connect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println("Disconnected");
  Serial.println("Advertising!");
}

void read_callback(BLECharacteristic& chr, ble_gatts_evt_read_t * request){
  Serial.println("\n****************\nRead Callback\n");
  if (chr.uuid == temperature_characteristic.uuid){
    Serial.println("UUID matches temperature uuid");
  }else if (chr.uuid == humidity_characteristic.uuid){
    Serial.println("UUID matches humidity uuid");
  }
}
//  #ifdef BAROMETER_BMP280
//  Serial.println("*In Barometer_BMP280");
//  float pressure;
//  //get and print temperatures
//  Serial.print("Temp:\t\t ");
//  Serial.print(bmp280.getTemperature());
//  Serial.println("C"); // The unit for  Celsius because original arduino don't support speical symbols
//  
//  //get and print atmospheric pressure data
//  Serial.print("Pressure:\t ");
//  Serial.print(pressure = bmp280.getPressure());
//  Serial.println("Pa");

void cccd_callback(BLECharacteristic& chr, uint16_t cccd_value)
{
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.println(cccd_value);
    Serial.println("");
    uint8_t nr_of_tries = 0;

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr.uuid == temperature_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Temperature Measurement 'Notify' enabled");
            float BLE_temperature = NULL;
            nr_of_tries = 0;
            while ((BLE_temperature == NULL) && (nr_of_tries < 3)){
              #ifdef BAROMETER_BMP280
                BLE_temperature = bmp280.getTemperature();
              #else if TEMPHUM_DHT
                BLE_temperature = dht.readTemperature();
              #endif
              if (BLE_temperature > 0 && BLE_temperature < 40){
                notifyTemperature(BLE_temperature*100);
                Serial.print("Sent temperature:\t");Serial.println(BLE_temperature);
              }else{
                Serial.print("****************\nfailed to get temperature");Serial.println(BLE_temperature);
                
                BLE_temperature = NULL;
              }
              nr_of_tries++;
            }
        } else {
            Serial.println("Temperature Measurement 'Notify' disabled");
        }
    }
    else if (chr.uuid == humidity_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Humidity Measurement 'Notify' enabled");
            float BLE_humidity = NULL;
            nr_of_tries = 0;
            while ((BLE_humidity == NULL) && (nr_of_tries < 3)){
              Serial.println("In While loop");
              BLE_humidity = dht.readHumidity();
              if (BLE_humidity){
                notifyHumidity(BLE_humidity*100);
                Serial.print("Sent humidity:\t");Serial.println(BLE_humidity);
              }else{
                nr_of_tries++;
                Serial.println("****************\nfailed to get humidity");
              }
              
              
            }
        } else {
            Serial.println("Humidity Measurement 'Notify' disabled");
        }
    }
    else if (chr.uuid == pressure_characteristic.uuid) {
        if (chr.notifyEnabled()) {
            Serial.println("Pressure Measurement 'Notify' enabled");
            float BLE_pressure = NULL;
            nr_of_tries = 0;
            while ((BLE_pressure == NULL) && (nr_of_tries < 3)){
              Serial.println("In While loop");
              BLE_pressure = bmp280.getPressure();
              if (BLE_pressure){
                notifyPressure(BLE_pressure*10);
                Serial.print("Sent pressure:\t");Serial.println(BLE_pressure);
              }else{
                nr_of_tries++;
                Serial.println("****************\nfailed to get pressure");
              }
              
              
            }
        } else {
            Serial.println("Humidity Measurement 'Notify' disabled");
        }
    }
}

//void loop()
//{
//  digitalToggle(LED_RED);
//  
//  if ( Bluefruit.connected() ) {
//    int16_t temperature = (int16_t)dht.readTemperature();
//    //uint8_t hrmdata[2] = { 0b00000110, bps };           // Sensor connected, increment BPS value
////    uint8_t hrmdata[2] = {bps>>8,bps | 0b11111111};//{ 0b00000110, bps };           // Sensor connected, increment BPS value
//    
//    
//    // Note: We use .notify instead of .write!
//    // If it is connected but CCCD is not enabled
//    // The characteristic's value is still updated although notification is not sent
//    //if ( temperature_characteristic.notify(hrmdata, sizeof(hrmdata)) ){
//    if ( temperature_characteristic.notify32(temperature) ){
//      Serial.print("Heart Rate Measurement updated to: "); Serial.println(temperature); 
//    }else{
//      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
//    }
//  }
//
//  // Only send update once per second
//  delay(1000);
//}

/**
 * RTOS Idle callback is automatically invoked by FreeRTOS
 * when there are no active threads. E.g when loop() calls delay() and
 * there is no bluetooth or hw event. This is the ideal place to handle
 * background data.
 * 
 * NOTE: FreeRTOS is configured as tickless idle mode. After this callback
 * is executed, if there is time, freeRTOS kernel will go into low power mode.
 * Therefore waitForEvent() should not be called in this callback.
 * http://www.freertos.org/low-power-tickless-rtos.html
 * 
 * WARNING: This function MUST NOT call any blocking FreeRTOS API 
 * such as delay(), xSemaphoreTake() etc ... for more information
 * http://www.freertos.org/a00016.html
 */
//void rtos_idle_callback(void)
//{
//  // Don't call any other FreeRTOS blocking API()
//  // Perform background task(s) here
//}
#endif
