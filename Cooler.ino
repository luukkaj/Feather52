#ifdef COOLER

#include "Arduino.h"
#include "Wire.h"
#include <bluefruit.h>
#include <Adafruit_PWMServoDriver.h>
#include "math.h"

#define FAN_PWMPIN    2
#define COOLER_PWMPIN 3
#define TEMP_PIN      A2

#define FAN_MIN       800
#define FAN_MAX       4000


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

void coolerSetup() {
  //Serial.begin(9600);
  //Wire.setClock(10000);
  //Wire.begin();
  pwm.setPWM(COOLER_PWMPIN, 4096, 0);
  pwm.begin();
  pinMode( TEMP_PIN, INPUT);

  TaskHandle_t TaskHandle_Cooler;
  xTaskCreate(
    vTaskCooler,
    "coolerTask",
    700,
    NULL,
    1,
    &TaskHandle_Cooler);

}

float getDewPoint(){//float lastDewPoint){
  float RH = getHumidity();
  float T = getTemperature();

  float dew_point = 243.04*(log(RH/100.0)+((17.625*T)/(243.04+T)))/(17.625-log(RH/100.0)-((17.625*T)/(243.04+T)));
  return dew_point;
}

float getCoolerSetPoint(){
  float RH = getHumidity()*0.9;
  float T = getTemperature()*0.9;

  float dew_point = 243.04*(log(RH/100.0)+((17.625*T)/(243.04+T)))/(17.625-log(RH/100.0)-((17.625*T)/(243.04+T)));
  return dew_point-2;
}

void turnCoolerOn(void){
  pwm.setPWM(COOLER_PWMPIN, 0, 4096);
}

void turnCoolerOff(void){
  pwm.setPWM(COOLER_PWMPIN, 4096, 0);
}

float getCoolerTemperature(void){
  float voltage_reading = 0;
  uint8_t nr_of_loops = 10;
  for (uint8_t i = 0; i < nr_of_loops ; i++){
    voltage_reading += analogRead(TEMP_PIN);
  }
  float voltage = (voltage_reading/nr_of_loops)*3600/1024.0;
  if(isnan(voltage)){
    Serial.println("VOLTAGE NOT VALID");
    Serial.print("voltage reading:  ");Serial.println(voltage_reading);
    Serial.print("Voltage: ");Serial.println(voltage);
  }
  return ((voltage - 500) / 10.0 );
}

static void vTaskCooler(void* pvParameters){
  turnCoolerOff();
  delay(2000);
  turnCoolerOn();
  delay(2000);
  float lastTempError = 0;
  float tempError = 0;
  float integral = 0;
  float derivator = 0;
  uint16_t loopTime = 100;

//  float Kp = 12;//60;
//  float Ki = 0.04;//0.02;
//  float Kd = 200;//1800;
  uint8_t Kp = 180;//60;
  float Ki = 0.01;//0.02;
  uint16_t Kd = 3000;//1800;
//  float Kp = 4.5;
//  float Ki = 0.00001;//0.05;
//  float Kd = 300;//150;//50;//3;
//  int16_t Kp = 6.6;   // 0,6*Ku
//  int16_t Ki = 0.5;//0.044;  // 2Kp/Pu
//  int16_t Kd = 247;   // Kp*Pu/8

  int16_t output = 1000;//FAN_MIN;
  uint8_t loopNr = 0;
  int32_t max_integral = 3000/Ki;
  float goal_temp = getDewPoint()-3;//(-1);
  float cooler_temp = getCoolerTemperature();
  int16_t dewPoint = getDewPoint()*100;
  uint32_t writeTime = millis();
  UBaseType_t uxHighWaterMark;

        /* Inspect our own high water mark on entering the task. */
  
  while (1){
    /*
     * Humidity read fails as it contains delays allowing switching of task
     * Change to deasent sensor, remove delays or disable interrupts
     */
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    
    loopNr++;
    
    cooler_temp = 0.1*getCoolerTemperature() + 0.9*cooler_temp;
    
    tempError = goal_temp-cooler_temp;
    derivator = (tempError - lastTempError)/loopTime;

     if ((isnan(tempError)) || (isnan(loopTime))){
        Serial.println("\n****************************\nTempError or loopTime not valid!");
        Serial.print("Integral:  ");Serial.println(integral);
        Serial.print("TempError: ");Serial.println(tempError);
        Serial.print("loopTime:  ");Serial.println(loopTime);
      }
    
    if (integral <= -max_integral){
      integral += 10;
    }else if (integral  >= max_integral){
      integral -= 10;
    }else if ((!((tempError > 0) && (output >= FAN_MAX))) && (!((tempError < 0) && (output <= FAN_MIN))) && (!(isnan(tempError)))){
      integral += tempError * loopTime;
    }
    
    if (isnan(integral)){
        Serial.println("\n****************************\nIntegral not valid!");
        Serial.print("Integral:  ");Serial.println(integral);
        Serial.print("TempError: ");Serial.println(tempError);
        Serial.print("loopTime:  ");Serial.println(loopTime);
        integral = 0;
      }
    
    output = Kp * tempError + Ki * integral + Kd * derivator + 1400;
    lastTempError = tempError;//0.5*tempError + 0.5*lastTempError;
    
    if (output <= FAN_MIN){
      output = 0;     
    }else if (output >= FAN_MAX){
      output = FAN_MAX;
    }
    dewPoint = getDewPoint()*100;
    if (writeTime < millis()){//(loopNr >= 20){
      uint16_t temp = 10*getTemperature();
      uint16_t hum = 10*getHumidity();

      Serial.print("  Temp: ");
//      if (cooler_temp < -10){
//        Serial.print(-10);
//      }else {
        Serial.print(cooler_temp);
      //}
      Serial.print("/");
//      if (goal_temp > 10){
//        Serial.print(10);
//      }else if (goal_temp < -10){
//        Serial.print(-10);
//      }else{
        Serial.print(goal_temp);
//      }
      Serial.print("\t Output: ");
      Serial.print(output/1000.0);
      Serial.print("\tChamber temp: ");
      Serial.print(temp/100.0);
      Serial.print("\tChamber hum: ");
      Serial.print(hum/100.0);
//      if (antiWindup == true){
//        Serial.print(3);
//      }else{
//        Serial.print(-3);
//      }
      Serial.print("\tDewPoint: ");
      
//      if (dewPoint < -1000){
//        dewPoint = -1000;
//      }else if (dewPoint > 600){
//        dewPoint = 600;
//      }
      Serial.print(dewPoint/100.0);
      Serial.print("\tIntegral: ");
      Serial.println((integral*Ki)/1000.0);
      Serial.print("Remaining stack size: ");Serial.println(uxHighWaterMark);
      
      //loopNr = 0;
      writeTime = millis() + 15*1000;
    }

    goal_temp = getCoolerSetPoint();//(dewPoint/100.0)-3.5;
    pwm.setPWM(FAN_PWMPIN, 0, (uint16_t)output);
    delay(loopTime);
  }
}

#endif //COOLER
