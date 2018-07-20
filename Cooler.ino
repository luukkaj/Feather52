#if COOLER

#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>
#include <Adafruit_PWMServoDriver.h>
#include "math.h"

#define FAN_PWMPIN        2
#define COOLER_PWMPIN     3
#define HUMIDIFYER_PWMPIN 0
#define TEMP_PIN          A2

#define FAN_MIN       800
#define FAN_MAX       4000

//#if (PWM_DEFINED == 0)
//#define PWM_DEFINED   1
//  Serial.println("Cooler defined PWMController");
//  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
//#endif

void coolerSetup() {
  //Serial.begin(9600);
  //Wire.setClock(10000);
  //Wire.begin();
  
  #if PWMSTARTED == 0
    #define PWMSTARTED 1
    if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
      pwm.begin();
      xSemaphoreGive(xI2CSemaphore);
    }else{
      Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in CoolerSetup\n\n");
    }
    //pwm.reset();
  #endif
  turnCoolerOff();
  pinMode( TEMP_PIN, INPUT);

  //TaskHandle_t TaskHandle_Cooler;
  xTaskCreate(
    vTaskCooler,
    "coolerTask",
    675,
    NULL,
    2,
    &TaskHandle_Cooler);

  //TaskHandle_t TaskHandle_Defrost;
  xTaskCreate(
    vTaskDefrost,
    "defrostTask",
    450+100,
    NULL,
    1,
    &TaskHandle_Defrost);

  #if HUMIDITY_CONTROL
    xTaskCreate(
      vTaskHumidify,
      "humidifyTask",
      500+100,
      NULL,
      1,
      &TaskHandle_Humidify);
    xSemaphoreGive(xHumidifySemaphore);
  #else
    turnHumidifyerOff();
  #endif

  xSemaphoreGive(xCoolerSemaphore);
}

//float getDewPoint(){//float lastDewPoint){
//  float RH = getHumidity();
//  float T = getTemperature();
//
//  float dew_point = 243.04*(log(RH/100.0)+((17.625*T)/(243.04+T)))/(17.625-log(RH/100.0)-((17.625*T)/(243.04+T)));
//  return dew_point;
//}

float getCoolerSetPoint(){
  float RH = getHumidity();
  float T = getTemperature();

  float dew_point = 243.04*(log(RH/100.0)+((17.625*T)/(243.04+T)))/(17.625-log(RH/100.0)-((17.625*T)/(243.04+T)));
  return dew_point-4;
}

void turnCoolerOn(void){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 10000)){
    pwm.setPWM(COOLER_PWMPIN, 0, 4096);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in turnCoolerOn\n\n");
  }
}
void turnCoolerOff(void){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 10000)){
    pwm.setPWM(COOLER_PWMPIN, 4096, 0);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in turnCoolerOn\n\n");
  }
}


void turnHumidifyerOn(void){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 10000)){
    pwm.setPWM(HUMIDIFYER_PWMPIN, 0, 4096);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in turnHumidifyerOn\n\n");
  }
}
void turnHumidifyerOff(void){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 10000)){
    pwm.setPWM(HUMIDIFYER_PWMPIN, 4096, 0);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in turnHumidifyerOff\n\n");
  }
}

float getCoolerTemperature(void){
  float voltage_reading = 0;
  uint8_t nr_of_loops = 3.0;
  for (uint8_t i = 0; i < nr_of_loops ; i++){
    voltage_reading += analogRead(TEMP_PIN);
    vTaskDelay(10);
  }
  //float voltage = (voltage_reading/nr_of_loops)*3600/1024.0;
  //float voltage = (voltage_reading/nr_of_loops)*3400/1024.0; // Operating voltage measured to 3,209V
  float voltage = (voltage_reading/nr_of_loops)*3238/1024.0; // Operating voltage measured to 3,209V
  if(isnan(voltage)){
    Serial.println("\n*******************\nVOLTAGE NOT VALID");
    Serial.print("voltage reading:  ");Serial.println(voltage_reading);
    Serial.print("Voltage: ");Serial.println(voltage);
    Serial.println("*********************************\n");
  }
  return ((voltage - 500) / 10.0 );
}

static void vTaskCooler(void* pvParameters){

  turnCoolerOff();
  delay(2000);
  turnCoolerOn();
  delay(2000);
  float lastTempError = 0;
  int32_t tempError = 0;
  float integral = 0;
  float derivator = 0;
  uint16_t loopTime = 300;


  uint8_t   Kp = 160;
  float     Ki = 0.01;
  uint16_t  Kd = 3000;


  int32_t   output        = FAN_MIN;
  uint8_t   loopNr        = 0;
  int32_t   max_integral  = 3000/Ki;
  float     goal_temp     = getCoolerSetPoint();
  float     new_goal_temp = 0;
  float     cooler_temp   = getCoolerTemperature();
  int32_t   dewPoint      = getDewPoint()*100;
  uint32_t  writeTime     = millis();

  uint32_t measuredLoopTime;
  
  UBaseType_t uxHighWaterMark;

        /* Inspect our own high water mark on entering the task. */
  
  while (1){
    if (xSemaphoreTake(xCoolerSemaphore, (TickType_t) 30000)){
      measuredLoopTime = millis();
      loopNr++;
  
      cooler_temp = 0.1*getCoolerTemperature() + 0.9*cooler_temp;
  
      if (isnan(cooler_temp)){
        Serial.print("\n\n------------------------------\nCooler temp not valid\n------------------\n\n");
      }
      
      tempError = goal_temp-cooler_temp;
      
  
      if (isnan(tempError)){
          tempError = 0;
          Serial.println("\n\n------------------------------\nTempErorr not valid and changed to 0\n------------------\n\n");
        }
      derivator = (tempError - lastTempError)/loopTime;
      
      // If output can and should be increased/decresed and tempError is a valid value
      if ((((tempError > 0) && (output < FAN_MAX)) || ((tempError < 0) && (output > FAN_MIN))) && (!(isnan(tempError)))){
        integral += tempError * loopTime;
      }else if (integral <= -max_integral){
        integral += 10;
      }else if (integral  >= max_integral){
        integral -= 10;
      }
      
      if (isnan(integral)){
          Serial.println("\n****************************\nIntegral not valid!");
          Serial.print("Integral:  ");Serial.println(integral);
          Serial.print("TempError: ");Serial.println(tempError);
          Serial.print("loopTime:  ");Serial.println(loopTime);
          integral = 0;
        }
      
      output = Kp * tempError + Ki * integral + Kd * derivator + 1400;
      lastTempError = tempError;
  
      if (isnan(output)){
        Serial.println("\n\n****************************\n****************************\n");
        Serial.println("Output not valid!");
        Serial.print("Kp: \t");Serial.println(Kp);
        Serial.print("tempError: \t");Serial.println(tempError);
        Serial.print(" -Goal Temp: \t");Serial.println(goal_temp);
        Serial.print(" -Cooler Temp: \t");Serial.println(cooler_temp);
        Serial.print("\nKi: \t");Serial.println(Ki);
        Serial.print("integral: \t");Serial.println(integral);
        Serial.print(" -tempError: \t");Serial.println(tempError);
        Serial.print(" -loopTime: \t");Serial.println(loopTime);
        Serial.print("Kd: \t");Serial.println(Kd);
        Serial.print("derivator:  ");Serial.println(derivator);
        Serial.print("-lastTempError:  ");Serial.println(lastTempError);
        Serial.println("****************************\n****************************\n\n");
      }
  
      
      if (output <= FAN_MIN){
        output = 0;     
      }else if (output >= FAN_MAX){
        output = FAN_MAX;
      }
      dewPoint = getDewPoint()*100;
      if (isnan(dewPoint)){
        Serial.print("\n\n**********************************\\tInvalid dewpoint value\n*******************************\n\n");
      }
      if (writeTime < millis()){
        uint16_t temp = 10*getTemperature();
        uint16_t hum = 10*getHumidity();
        vTaskDelay(10); // Allow other tasks to run before timeconsuming print
        uint32_t printTime = millis();
        Serial.print("\n\tTemp: ");
        Serial.print(cooler_temp);
        Serial.print("/");
        Serial.print(goal_temp);
        Serial.print("\t Output: ");
        Serial.print(output/1000.0);
        Serial.print("\tChamber temp: ");
        Serial.print(temp/100.0);
        Serial.print("\tChamber hum: ");
        Serial.print(hum/100.0);
        Serial.print("\tDewPoint: ");
        Serial.print(dewPoint/100.0);
        Serial.print("\tIntegral: ");
        Serial.println((integral*Ki)/1000.0);
        uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
        Serial.print("\tRemaining stack size: ");Serial.println(uxHighWaterMark);
        
        Serial.print("\tMeasured time for cooler loop/print:\t");Serial.print(printTime-measuredLoopTime);Serial.print("/");Serial.print(millis()-printTime);Serial.println(" milliseconds");//550millis
        writeTime = millis() + 20*1000;
      }
  
      new_goal_temp = getCoolerSetPoint();
      if (isnan(new_goal_temp)){
        Serial.println("\n\n********************\nInvalid goal temperature\nSet to last goal_temp\n\n");
      }else{
        goal_temp = new_goal_temp;
      }
  
      if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
          pwm.setPWM(FAN_PWMPIN, 0, (uint16_t)output);
          xSemaphoreGive(xI2CSemaphore);
        }else{
          Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in turnCoolerOn\n\n");
        }

      Serial.print(".");
      xSemaphoreGive(xCoolerSemaphore);
    }
    vTaskDelay(loopTime);
  }
}


static void vTaskDefrost(void* pvParameters){
  uint32_t milli_to_hours = 1000*60*60;
  while(1){
    vTaskDelay(3*milli_to_hours);
    // Wait for I2C semaphore in order not to suspend Cooler Task while it has the semaphore
    //if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 10000)){
      //Serial.println("  *Obtained i2c Semaphore");
      if (xSemaphoreTake(xCoolerSemaphore, (TickType_t) 60000)){
        Serial.println("\n  *************************************");
        Serial.println("  *Obtained Cooler Semaphore");
        Serial.println("  *************************************\n");
        #if HUMIDITY_CONTROL
          Serial.println("Waiting for humidify semaphore");
          if (xSemaphoreTake(xHumidifySemaphore, (TickType_t) (15*60*1000))){
            Serial.println("  *Obtained Humidify Semaphore");
        #endif
//          Serial.println("Waiting for tension semaphore");
//          if (xSemaphoreTake(xTensionSemaphore, (15*60*1000))){
//            Serial.println("\n  *************************************");
//            Serial.println("  *Obtained tension Semaphore");
//            Serial.println("  *************************************\n");
            //vTaskSuspend(TaskHandle_Cooler);
            //Trurning off cooler will turn off power for tension measurement
            
            //vTaskSuspend(TaskHandle_Tension);
            //#if HUMIDITY_CONTROL
            //  vTaskSuspend(TaskHandle_Humidify);
            //#endif
            //Semaphore is no longer neccessary and can be given back
            //xSemaphoreGive(xI2CSemaphore);
    
            turnCoolerOff();
            Serial.println("\n\n-------------------------------------------------\n\tStarted Defrosting\n-------------------------------------------------\n\n");
            while((getCoolerTemperature()) < 20){
              vTaskDelay(15*1000);
            }
            vTaskDelay(60*1000); // Turn off cooler for 1 min too long
            Serial.println("\n\n-------------------------------------------------\n\tDefrosting Finnished\n-------------------------------------------------");
            Serial.print("Cooler Temperature after defrosting: ");Serial.print(getCoolerTemperature());Serial.println("\n\n");
            turnCoolerOn();
    

  //          #if HUMIDITY_CONTROL
  //            vTaskResume(TaskHandle_Humidify);
  //          #endif
            xSemaphoreGive(xCoolerSemaphore);
            #if HUMIDITY_CONTROL
              xSemaphoreGive(xHumidifySemaphore);
            #endif
            //xSemaphoreGive(xTensionSemaphore);
//          }else{
//            Serial.println("\n  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//            Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: Could not suspend Tension task\n\n");
//            Serial.println("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
//          }
        #if HUMIDITY_CONTROL
          }else{
            Serial.println("\n  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: Could not suspend Humidify task\n\n");
            Serial.println("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
          }
        #endif
      }else{
        Serial.println("\n  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: Could not suspend Cooler task\n\n");
        Serial.println("  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
      }
//    }else{
//      Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: Could not suspend i2c traffic\n\n");
//    }
    
  }
}
static void vTaskHumidify(void* pvParameters){
  vTaskDelay(2000);
  turnHumidifyerOn();
  vTaskDelay(500);
  turnHumidifyerOff();
  while(1){
    vTaskDelay(90*60*1000);
    Serial.println("\n\n************************************\nStarted humidifying\n************************************\n");
    //turnCoolerOff();
    if (xSemaphoreTake(xHumidifySemaphore, (TickType_t) (5*60*1000))){
      turnHumidifyerOn();
      while(getHumidity() < 85){
  //      turnCoolerOff(); // If humidifyer turns the cooler back on
  //      turnHumidifyerOn();
        vTaskDelay(5*1000);
  //      turnHumidifyerOff();
  //      vTaskDelay(60*1000);
      }
      turnHumidifyerOff();
      xSemaphoreGive(xHumidifySemaphore);
    }else{
      Serial.println("Failed to obtain senaphore and could not humidify");
    }
//    vTaskDelay(5*60*1000);
//    turnCoolerOn();
    Serial.print("Humidity: ");Serial.println(getHumidity());
    //turnHumidifyerOff();
    Serial.println("\n\n************************************\nHumidifyer stopped\n\n\n************************************\n");
  }
}


#endif //COOLER
