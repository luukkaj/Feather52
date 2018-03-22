#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>
#include "customGATT.h"
#include "semphr.h"

/****************************
 * Include sensors
 */
#define BAROMETER_BMP280  1
#define TEMPHUM_DHT       1
#define TEMPHUM_TH02      0
#define VOC_COREP         1
#define ENABLE_BLUETOOTH  1
//#define TENSION           0
#define COOLER_TEMP     1



#if TEMPHUM_DHT | BAROMETER_BMP280
  #define TEMPERATURE
#endif
#if TEMPHUM_DHT | TEMPHUM_TH02
  #define HUMIDITY
#endif
#if BAROMETER_BMP280
  #define PRESSURE
#endif
#if  VOC_COREP
  #define VOC           1
#endif
#define COOLER        1
#if (defined(TEMPERATURE) && defined(HUMIDITY))
  #define DEW_POINT
#endif



#define WRITE_INTERVAL    10000 //milliseconds
#define NOTIFY_INTERVAL   2000  //milliseconds




#ifdef ENABLE_BLUETOOTH
  // Advanced function prototypes
  //void startAdv(void);
  //void setupHRM(void);
  //void connect_callback(uint16_t conn_handle);
  //void disconnect_callback(uint16_t conn_handle, uint8_t reason);
#endif

#ifdef BAROMETER_BMP280
  #include <Seeed_BMP280.h>
  BMP280 bmp280;
#endif

#ifdef TEMPHUM_DHT
  #include <DHT.h>
  #define DHTPIN A1
  DHT dht(DHTPIN, DHT22, 32);
#endif

static void vTaskNotifyBLE( void* pvParameters);


void setup()
{
  Serial.begin(9600);
  Wire.setClock(10000);
  Wire.begin();
  //Serial.println("0 0 0 0 0 0 0");
  Serial.println("Setting up...");
  
  
  #if BAROMETER_BMP280
    if(!bmp280.init()){
      Serial.println("Device error!");
    }
    delay(100);
  #endif //BAROMETER_BMP280
  
  
  #if TEMPHUM_DHT
    dht.begin();
    delay(150); // Delay for the temperature humidity sensor to stabilize
  #endif

  #if TENSION
    tensionSetup();
  #endif

  #if COOLER
    coolerSetup();
  #endif
  
  #if ENABLE_BLUETOOTH
    xTaskCreate(
      vTaskNotifyBLE,
      "notifyTask",
      400,
      NULL,
      1,
      &TaskHandle_Notify);
  
    
    xTaskCreate(
      vTaskWriteBLE,
      "writeTask",
      600,
      NULL,
      2,
      &TaskHandle_Write);
  
    setupBluetooth();
    xSemaphoreGive( xBLEWriteSemaphore );  
  #endif //ENABLE_BLUETOOTH

  //Wire.begin();
  Serial.println("Setup done");
}


static void vTaskNotifyBLE(void* pvParameters)
{
  uint8_t notifyLoop = 0;
  UBaseType_t uxHighWaterMark;
  bool notification_sent = false;
  while(1){
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    if ( Bluefruit.connected() ) {
    
      if (temperature_characteristic.notifyEnabled()){
        notifyTemperature(getTemperature());
        notification_sent = true;
      }
      if (humidity_characteristic.notifyEnabled()){
        notifyHumidity(getHumidity());
        notification_sent = true;
      }
      if (pressure_characteristic.notifyEnabled()){
        notifyPressure(getPressure());
        notification_sent = true;
      }
      if (dew_point_characteristic.notifyEnabled()){
        notifyDewPoint(getDewPoint());
        notification_sent = true;
      }
      if (cooler_temp_characteristic.notifyEnabled()){
        notifyDewPoint(getCoolerTemperature());
        notification_sent = true;
      }
    }
    if (notification_sent){
      Serial.println("");
      notification_sent = false;
    }
    vTaskDelay(NOTIFY_INTERVAL);
    if (notifyLoop == 250){
      Serial.print("Notify remaining stack size: ");Serial.println(uxHighWaterMark);
      notifyLoop = 0;
    }
    //vTaskSuspend(NULL);
    notifyLoop++;
  }
}


static void vTaskWriteBLE(void* pvParameters)
{
  UBaseType_t uxHighWaterMark;
  
  vTaskSuspend(NULL);
  while(1){
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    while ( Bluefruit.connected() ) {
    
      #ifdef TEMPERATURE
        writeTemperature(getTemperature());
      #endif //Temperature
      
      #ifdef HUMIDITY
        writeHumidity(getHumidity());
      #endif // Humidity
  
      #ifdef PRESSURE
        writePressure(getPressure());
      #endif // Pressure
        
      #ifdef VOC
        writeTVOC(CoreP_getTVOC());
      #endif // VOC
  
      #ifdef DEW_POINT
        writeDewPoint(getDewPoint());
      #endif //DewPoint
      
      #ifdef COOLER_TEMP
        writeCoolerTemperature(getCoolerTemperature());
      #endif //CoolerTemp
      
      delay(WRITE_INTERVAL);
    }
    Serial.print("Write Task watermark: ");Serial.println(uxHighWaterMark);
    vTaskSuspend(NULL);
  }
}


int loop_nr = 0;
void loop()
{
  vTaskSuspend(NULL);
  unsigned long start_time = millis();
  while ((millis()-start_time) < 250){}
  Serial.print("Time to loop: ");Serial.println(millis()-start_time);
  
  
  }
//{
//  UBaseType_t uxHighWaterMark;
//  
//  #ifndef ENABLE_BLUETOOTH
//    /******************************************
//     * Barometer BMP280
//     */
//    #ifdef BAROMETER_BMP280
//    Serial.println("*In Barometer_BMP280");
//    //float pressure;
//    //get and print temperatures
//    Serial.print("Temp:\t\t ");
//    Serial.print(bmp280.getTemperature());
//    Serial.println("C"); // The unit for  Celsius because original arduino don't support speical symbols
//    
//    //get and print atmospheric pressure data
//    Serial.print("Pressure:\t ");
//    Serial.print(bmp280.getPressure());
//    Serial.println("Pa");
//    
//    //get and print altitude data
//    Serial.print("Altitude:\t ");
//    Serial.print(bmp280.calcAltitude(pressure));
//    Serial.println("m");
//    #endif // BAROMETER BMP280
//    /******************************************/
//
//
//    /******************************************
//     * VOC CoreP
//     */
//    #ifdef VOC_COREP
//    Serial.println("*In VOC_COREP");
//    int co2_value = CoreP_getCO2();
//    int tvoc_value = CoreP_getTVOC();
//  
//    Serial.print("CO2 value:\t ");Serial.println(co2_value);
//    Serial.print("TVOC value:\t ");Serial.println(tvoc_value);
//    #endif
//    /*****************************************/
//
//    /*****************************************
//     * Temperature Humidity DHT
//     */
//    #ifdef TEMPHUM_DHT
//    Serial.println("*In TEMPHUM_DHT");
//    float dht_humidity = dht.readHumidity();
//    float dht_temperature = dht.readTemperature();
//    Serial.print("DHT Temp value:\t\t ");Serial.print(dht_temperature);Serial.println("C");
//    Serial.print("DHT Humidity value:\t ");Serial.print(dht_humidity);Serial.println(" %RH");
//    #endif
//    /*****************************************/
//  #endif //EnableBluetooth
//
//
//
//  /*****************************************
//   * Bluetooth
//   */
//  #ifdef ENABLE_BLUETOOTH
//  //if ( Bluefruit.connected() ) {
//  if( xSemaphoreTake( xBLEWriteSemaphore, ( TickType_t ) 10000 ) == pdTRUE )
//  {
//    //unsigned long start_time = millis();
//    #ifdef TEMPERATURE
//      float temperature = getTemperature();
//      writeTemperature(getTemperature());
//    #endif //Temperature
//    
//    #ifdef HUMIDITY
//      float humidity = getHumidity();
//      writeHumidity(humidity);
//    #endif // Humidity
//
//    #ifdef PRESSURE
//      float pressure = getPressure();
//      writePressure(pressure);
//    #endif // Humidity
//      
//    #ifdef VOC
//      uint16_t tvoc = CoreP_getTVOC();
//      writeTVOC(tvoc);
//    #endif
//
//    #ifdef DEW_POINT
//      writeDewPoint(getDewPoint());
//    #endif
//    #ifdef COOLER_TEMP
//      writeCoolerTemperature(getCoolerTemperature());
//    #endif
//    
//  //unsigned long end_time = millis();
//  //Serial.print("Time to measure in milli seconds: {}");Serial.println(end_time-start_time);
//  #endif // ENABLE_BLUETOOTH
//
//  /*******************************************
//   * Water tension measurement
//   */
//  #ifdef TENSION
//  Serial.println(getWaterTension(5));
//  //getWaterTension(10);
//  //setPWM(1,1000);
//  //delay(200);
//  //setPWM(1,1100);
//  //delay(1000);
//  //setPWM(1,0);
//  #endif // Tension
//
//  
//
//  //Serial.println("\n");//add a line between output of different times.
//  //delay(WRITE_INTERVAL);
//  vTaskDelay(WRITE_INTERVAL);
//  }
//  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
//  loop_nr++;
//    Serial.print("main stack size: ");Serial.println(uxHighWaterMark);
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
void rtos_idle_callback(void)
{
  // Don't call any other FreeRTOS blocking API()
  // Perform background task(s) here
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
}
