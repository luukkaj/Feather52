#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>
#include "customGATT.h"

/****************************
 * Include sensors
 */
#define BAROMETER_BMP280  1
#define TEMPHUM_DHT       1
//#define TEMPHUM_TH02      0
#define VOC_COREP         1
#define ENABLE_BLUETOOTH  1
//#define TENSION           1

#define TEMPERATURE   TEMPHUM_DHT | BAROMETER_BMP280
#define HUMIDITY      TEMPHUM_DHT | TEMPHUM_TH02
#define PRESSURE      BAROMETER_BMP280
#define VOC           VOC_COREP

#define WRITE_INTERVAL    15000 //milliseconds
#define NOTIFY_INTERVAL   2000  //milliseconds


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

static void vTaskNotifyBLE( void* pvParameters);


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

  TaskHandle_t TaskHandle_Notify;
  xTaskCreate(
    vTaskNotifyBLE,
    "notifyTask",
    800,
    NULL,
    1,
    &TaskHandle_Notify);
    
  
  Serial.println("Setup done");
}


static void vTaskNotifyBLE(void* pvParameters)
{
  bool notification_sent = false;
  while(1){
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
    }
    if (notification_sent){
      Serial.println("");
      notification_sent = false;
    }
    vTaskDelay(NOTIFY_INTERVAL);
  }
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

    #ifdef TEMPERATURE
      float temperature = getTemperature();
      writeTemperature(temperature);
    #endif //Temperature
    
    #ifdef HUMIDITY
      float humidity = getHumidity();
      writeHumidity(humidity);
    #endif // Humidity

    #ifdef PRESSURE
      float pressure = getPressure();
      writePressure(pressure);
    #endif // Humidity
      
    #ifdef VOC
      float tvoc = CoreP_getTVOC();
      writeTVOC(tvoc);
    #endif
    

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
  //delay(WRITE_INTERVAL);
  vTaskDelay(WRITE_INTERVAL);
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
  //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
}
