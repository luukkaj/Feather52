#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>
#include "customGATT.h"
#include "semphr.h"
#include "task.h"
SemaphoreHandle_t xI2CSemaphore;

/****************************
 * Include sensors
 */
#define BATTERY_DRIVEN    1
#define HUMIDITY_CONTROL  0

#if BATTERY_DRIVEN
  #define BAROMETER_BMP280  1
  #define TEMPHUM_DHT       0
  #define TEMPHUM_TH02      0
  #define SHT31             1
  #define VOC_COREP         1
  #define ENABLE_BLUETOOTH  1
  #define TENSION           0
  #define COOLER_TEMP       0
  #define COOLER            0
#else
  #define BAROMETER_BMP280  1
  #define TEMPHUM_DHT       0
  #define TEMPHUM_TH02      0
  #define SHT31             1
  #define VOC_COREP         1
  #define ENABLE_BLUETOOTH  1
  #define TENSION           1
  #define COOLER_TEMP       1
  #define COOLER            1
#endif


#define PWMSTARTED        0
#define PWM_DEFINED       0


#if TEMPHUM_DHT | BAROMETER_BMP280 | SHT31
  #define TEMPERATURE     1
#endif
#if TEMPHUM_DHT | TEMPHUM_TH02 | SHT31
  #define HUMIDITY        1
#endif
#if BAROMETER_BMP280
  #define PRESSURE        1
#endif
#if  VOC_COREP
  #define VOC             1
#endif

#if (defined(TEMPERATURE) && defined(HUMIDITY))
  #define DEW_POINT       1
#endif

#if (COOLER | TENSION) == 1
  #include <Adafruit_PWMServoDriver.h>
  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
  //Wire.begin();
#endif

#define WRITE_INTERVAL    15000 //milliseconds
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

const int buttonPin = 11;
void setup()
{
  //Button for triggering surface tension measurement
  pinMode(buttonPin, INPUT);
  

  Serial.begin(9600);
  
  //Serial.println("0 0 0 0 0 0 0");
  Serial.println("Setting up...");
  #if BAROMETER_BMP280 | TENSION | COOLER
    Wire.setClock(10000);
    Wire.begin();
    xI2CSemaphore = xSemaphoreCreateMutex();
    xSemaphoreGive(xI2CSemaphore);
  #endif
  #if BAROMETER_BMP280
    if(!bmp280.init()){
      Serial.println("Device error!");
    }
    //delay(100);
    Serial.println("BMP280 initialized");
  #endif //BAROMETER_BMP280
  
  
  #if TEMPHUM_DHT
    dht.begin();
    //delay(150); // Delay for the temperature humidity sensor to stabilize
    Serial.println("DHT initialized");
  #endif

  #if TENSION
    tensionSetup();
    Serial.println("Tension initialized");
  #endif

  #if COOLER
    coolerSetup();
    Serial.println("Cooler initialized");
  #endif
  
  #if ENABLE_BLUETOOTH
  setupBluetooth();
    xTaskCreate(
      vTaskNotifyBLE,
      "notifyTask",
      450,
      NULL,
      2,
      &TaskHandle_Notify);
  
    
    xTaskCreate(
      vTaskWriteBLE,
      "writeTask",
      600,
      NULL,
      2,
      &TaskHandle_Write);
      
  
    
    //xSemaphoreGive( xBLEWriteSemaphore );
    Serial.println("Bluietooth initialized");
  #endif //ENABLE_BLUETOOTH


  Serial.println("Setup done");
}


static void vTaskNotifyBLE(void* pvParameters)
{
  UBaseType_t uxHighWaterMark;
  bool notification_sent = false;

  vTaskSuspend(NULL);
  while(1){
    if (xSemaphoreTake(xBLENotifySemaphore, (TickType_t) 15000)){
      notification_sent = true;
      while (notification_sent == true){
        notification_sent = false;
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        
        if ( Bluefruit.connected() ) {
          #if TEMPERATURE
          if (temperature_characteristic.notifyEnabled()){
            notifyTemperature(getTemperature());
            notification_sent = true;
          }
          #endif
          
          #if HUMIDITY
          if (humidity_characteristic.notifyEnabled()){
            notifyHumidity(getHumidity());
            notification_sent = true;
          }
          #endif
          
          #if PRESSURE
          if (pressure_characteristic.notifyEnabled()){
            notifyPressure(getPressure());
            notification_sent = true;
          }
          #endif
          
          #if DEW_POINT
          if (dew_point_characteristic.notifyEnabled()){
            notifyDewPoint(getDewPoint());
            notification_sent = true;
          }
          #endif
        }
        if (notification_sent){
          Serial.println("");
          //notification_sent = false;
        }
        vTaskDelay(NOTIFY_INTERVAL);
      }
      Serial.print("Notify remaining stack size: ");Serial.println(uxHighWaterMark);
    //vTaskSuspend(NULL);
    }
  }
}





static void vTaskWriteBLE(void* pvParameters)
{
  UBaseType_t uxHighWaterMark;
  uint32_t measuredLoopTime;

  float temperature       = 0;
  float humidity          = 0;
  float pressure          = 0;
  uint16_t tvoc           = 0;
  float dewPoint          = 0;
  float coolerTemperature = 0;
  
  
  while(1){
    //vTaskSuspend(NULL);
    if (xSemaphoreTake(xBLEWriteSemaphore, (TickType_t) 15000)){
      while ( Bluefruit.connected() ) {
        measuredLoopTime = millis();
        uint32_t StartTime = 0;
        uint32_t EndTime = 0;
        uint32_t MiddleTime = 0;
        #ifdef TEMPERATURE
          //StartTime = millis();
          temperature = getTemperature();
          //MiddleTime = millis();
          if (!isnan(temperature)){
            writeTemperature(temperature);
          }
        //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        vTaskDelay(10);
        #endif //Temperature
        
        #ifdef HUMIDITY
          //StartTime = millis();
          humidity = getHumidity();
          //MiddleTime = millis();
          if (!isnan(humidity)){
            writeHumidity(humidity);
          }
          //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        vTaskDelay(10);
        #endif // Humidity
    
        #ifdef PRESSURE
          //StartTime = millis();
          pressure = getPressure();
          //MiddleTime = millis();
          if (!isnan(pressure)){
            writePressure(getPressure());
          }
          //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        vTaskDelay(10);
        #endif // Pressure
          
        #ifdef VOC
          //StartTime = millis();
          tvoc = CoreP_getTVOC();
          //MiddleTime = millis();
          if (!isnan(tvoc)){
            writeTVOC(CoreP_getTVOC());
          }
          //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        vTaskDelay(10);
        #endif // VOC
    
        #ifdef DEW_POINT
          //StartTime = millis();
          dewPoint = getDewPoint();
          //MiddleTime = millis();
          if (!isnan(dewPoint)){
            writeDewPoint(dewPoint);
          }
          //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        vTaskDelay(10);
        #endif //DewPoint
        
        #if COOLER_TEMP
          //StartTime = millis();
          coolerTemperature = getCoolerTemperature();
          //MiddleTime = millis();
          if (!isnan(dewPoint)){
            writeCoolerTemperature(getCoolerTemperature());
          }
          //EndTime = millis();
        //Serial.print("Measurement took: ");Serial.print(MiddleTime-StartTime);Serial.println(" milliseconds");
        //Serial.print("Sending took:     ");Serial.print(EndTime-MiddleTime);Serial.println(" milliseconds");
        #endif //CoolerTemp
        Serial.print("Measured time for write loop:\t");Serial.print(millis()-measuredLoopTime);Serial.println(" milliseconds");
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("Write Task watermark: ");Serial.println(uxHighWaterMark);
        delay(WRITE_INTERVAL);
      }
      if (!( Bluefruit.connected() )){
        Serial.println("\n Bluefruit.connected() == False\n");
      }
    }
  }
}


//int loop_nr = 1;
//int buttonState = 0;
//
//int iterations = 10;
//int measurements = 10;
//int tensioValue = 0;

void loop()
{
//  buttonState = digitalRead(buttonPin);
//  
//  if (( buttonState == HIGH)){
//    vTaskDelay(200);
//    setPWM(1, tensioValue);
//    tensioValue += 100;
//    vTaskDelay(50);
////    Serial.print("Measurement nr: ");Serial.println(loop_nr);
////    int i = 0;
////    while (i < iterations){
////      getWaterTension(measurements);
////      i++;
////    }
////    loop_nr++;
//  }
  //testPWM(1);
  
//  #if SHT31
//    //SHT31_getMeasurement();
//    Serial.print("Temperature: ");Serial.println(SHT31_getTemperature());
//    Serial.print("Humidity: ");Serial.println(SHT31_getHumidity());
//    vTaskDelay(1000);
//  #else
//  #endif
  vTaskDelay(5*60*1000);
//  Serial.println("\nRemaingin stack sizes:");
//  Serial.print("  - Write task:   ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Write));Serial.print(" / ");Serial.println(600*4);
//  Serial.print("  - Notify task:  ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Notify));Serial.print(" / ");Serial.println(400*4);
//  Serial.print("  - Tension task: ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Tension));Serial.print(" / ");Serial.println(690*4);
//  Serial.print("  - Cooler task:  ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Cooler));Serial.print(" / ");Serial.println(690*4);
//  Serial.print("  - Defrost task: ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Defrost));Serial.print(" / ");Serial.println(400*4);
//  Serial.print("  - BLE task:     ");Serial.print(uxTaskGetStackHighWaterMark(ble_task_hdl));Serial.print(" / ");Serial.println(512*3*4);
//  Serial.print("  - SOC task:     ");Serial.print(uxTaskGetStackHighWaterMark(soc_task_hdl));Serial.print(" / ");Serial.println(200*4);
//  Serial.print("  - IDLE task:    ");Serial.println(uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle()));
//  Serial.print("  - Callback task:");Serial.println(uxTaskGetStackHighWaterMark(callback_task_hdl));
  //Serial.print("\n  - Available heap: ");Serial.print(xPortGetMinimumEverFreeHeapSize());Serial.println("\n");

  Serial.println("\nvTaskList");
  char pcWriteBuffer [480];
  vTaskList(pcWriteBuffer);
  Serial.println("Name\tstate\tprio\tsize\tnr");
  Serial.println(pcWriteBuffer);
  Serial.println("\n");
  
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
  #if (COOLER | TENSION) == 0
    //sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    waitForEvent();
  #endif
}

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName ){

  Serial.println("\n\n\n*********************************************\n\tTask overflown\n*********************************************\n");
  Serial.print("Task name:      ");Serial.println(*pcTaskName);
  
  Serial.println("\nvTaskList");
  char pcWriteBuffer [480];
  vTaskList(pcWriteBuffer);
  Serial.println("Name\tstate\tprio\tsize\tnr");
  Serial.println(pcWriteBuffer);
  Serial.println("\n");

//  Serial.println("\nRemaingin stack sizes:");
//  Serial.print("  - Write task:   ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Write));Serial.print(" / ");Serial.println(600*4);
//  Serial.print("  - Notify task:  ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Notify));Serial.print(" / ");Serial.println(400*4);
//  Serial.print("  - Tension task: ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Tension));Serial.print(" / ");Serial.println(730*4);
//  Serial.print("  - Cooler task:  ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Cooler));Serial.print(" / ");Serial.println(730*4);
//  Serial.print("  - Defrost task: ");Serial.print(uxTaskGetStackHighWaterMark(TaskHandle_Defrost));Serial.print(" / ");Serial.println(600*4);
//  Serial.print("  - BLE task:     ");Serial.print(uxTaskGetStackHighWaterMark(ble_task_hdl));Serial.print(" / ");Serial.println(730*4);
//  Serial.print("  - SOC task:     ");Serial.print(uxTaskGetStackHighWaterMark(soc_task_hdl));Serial.print(" / ");Serial.println(600*4);
//  Serial.print("  - IDLE task:    ");Serial.print(uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle()));Serial.print(" / ");Serial.println(600*4);
//  Serial.print("  - Callback task:");Serial.print(uxTaskGetStackHighWaterMark(callback_task_hdl));Serial.print(" / ");Serial.println(600*4);
  
  while(1){ 
    Serial.print("*");  
  }
}

void vApplicationMallocFailedHook(void)
  {
    Serial.println("\n\n\tFailed to Malloc\n\n");
    while(1){ 
      Serial.print("Malloc Stucks  ");  
    }
  }

