#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>

/****************************
 * Include sensors
 */
#define SCAN_INTERVAL   30 //seconds
#define BAROMETER_BMP280  1
#define TEMPHUM_DHT       1
//#define TEMPHUM_TH02      0
//#define VOC_COREP         1
#define ENABLE_BLUETOOTH  1
//#define TENSION           1

#define TEMPERATURE   TEMPHUM_DHT | BAROMETER_BMP280
#define HUMIDITY      TEMPHUM_DHT | TEMPHUM_TH02
#define PRESSURE      BAROMETER_BMP280


#ifdef ENABLE_BLUETOOTH
  // Advanced function prototypes
  void startAdv(void);
  void setupHRM(void);
  void connect_callback(uint16_t conn_handle);
  void disconnect_callback(uint16_t conn_handle, uint8_t reason);
#endif


#ifdef BAROMETER_BMP280
  #include <Seeed_BMP280.h>
  BMP280 bmp280;
#endif

//#include <TH02_dev.h> Shit sensor don't use

#ifdef TEMPHUM_DHT
  #include <DHT.h>
  #define DHTPIN A1
  #define DHTTYPE DHT22
  DHT dht(DHTPIN, DHTTYPE, 32);
#endif




void setup()
{
  Serial.begin(9600);
  digitalWrite(20, LOW);
  digitalWrite(21,LOW);
  Wire.setClock(10000);
  Wire.begin();
  Serial.println("Setting up...");
  delay(150); // Delay for the temperature humidity sensor to stabilize
  
  #ifdef BAROMETER_BMP280
    if(!bmp280.init()){
      Serial.println("Device error!");
    }
  #endif
  //TH02.begin();
  delay(100);
  
  #ifdef TEMPHUM_DHT
    dht.begin();
  #endif

  #ifdef TENSION
    tensionSetup();
  #endif
  
  #ifdef ENABLE_BLUETOOTH
    setupBluetooth();
  #endif

  
  
  Serial.println("Setup done");
}



int loop_nr = 0;
void loop()
{


  #ifndef ENABLE_BLUETOOTH
  /******************************************
   * Barometer BMP280
   */
  #ifdef BAROMETER_BMP280
  Serial.println("*In Barometer_BMP280");
  float pressure;
  //get and print temperatures
  Serial.print("Temp:\t\t ");
  Serial.print(bmp280.getTemperature());
  Serial.println("C"); // The unit for  Celsius because original arduino don't support speical symbols
  
  //get and print atmospheric pressure data
  Serial.print("Pressure:\t ");
  Serial.print(pressure = bmp280.getPressure());
  Serial.println("Pa");
  
  //get and print altitude data
  Serial.print("Altitude:\t ");
  Serial.print(bmp280.calcAltitude(pressure));
  Serial.println("m");
  #endif
  /******************************************/


  /******************************************
   * VOC CoreP
   */
  #ifdef VOC_COREP
  Serial.println("*In VOC_COREP");
  int co2_value = CoreP_getCO2();
  int tvoc_value = CoreP_getTVOC();

  Serial.print("CO2 value:\t ");Serial.println(co2_value);
  Serial.print("TVOC value:\t ");Serial.println(tvoc_value);
  #endif
  /*****************************************/

  /*****************************************
   * Temperature Humidity DHT
   */
  #ifdef TEMPHUM_DHT
  Serial.println("*In TEMPHUM_DHT");
  float dht_humidity = dht.readHumidity();
  float dht_temperature = dht.readTemperature();
  Serial.print("DHT Temp value:\t\t ");Serial.print(dht_temperature);Serial.println("C");
  Serial.print("DHT Humidity value:\t ");Serial.print(dht_humidity);Serial.println(" %RH");
  #endif
  /*****************************************/
  #endif //EnableBluetooth



  /*****************************************
   * Bluetooth
   */
  #ifdef ENABLE_BLUETOOTH
  if ( Bluefruit.connected() ) {

    // Note: We use .notify instead of .write!
    // If it is connected but CCCD is not enabled
    // The characteristic's value is still updated although notification is not sent
    //if ( temperature_characteristic.notify(hrmdata, sizeof(hrmdata)) ){
    #ifdef TEMPERATURE
      float temperature = getTemperature();
      if (writeTemperature(temperature*100)){
        Serial.print("DHT Temp value:\t\t ");Serial.print(temperature);Serial.println("C");
        Serial.println("Temperature write successfull");
      }else{
        Serial.print("Temperature write failed. Value: ");Serial.print(temperature);Serial.println("C");
      }
    #endif //Temperature
    /*if ( notifyTemperature((uint16_t)BLE_temperature) ){
      Serial.print("Temperature Measurement updated to: "); Serial.println(BLE_temperature); 
    }else{
      Serial.println("Huaa ERROR: Notify not set in the CCCD or not connected!");
    }*/
    
    #ifdef HUMIDITY
      float humidity = getHumidity();
      if (writeHumidity(humidity*100)){
        Serial.print("DHT Humidity value:\t ");Serial.print(humidity);Serial.println(" %RH");
        Serial.println("Humidity write successfull");
      }else{
        Serial.print("Humidity write failed. Value:");
        Serial.print(humidity);Serial.println(" %RH");
      }
    #endif // Humidity
    /*if ( notifyHumidity((uint16_t)BLE_humidity) ){
      Serial.print("Humidity Measurement updated to: "); Serial.println(BLE_humidity); 
    }else{
      Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }*/
    
  //}
  #endif // ENABLE_BLUETOOTH

  /*******************************************
   * Water tension measurement
   */
  #ifdef TENSION
  Serial.println(getWaterTension(5));
  //getWaterTension(10);
  //setPWM(1,1000);
  //delay(200);
  //setPWM(1,1100);
  //delay(1000);
  //setPWM(1,0);
     
  #endif // Tension

  

  Serial.println("\n");//add a line between output of different times.
  delay(5000);
  }
}


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
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
}
