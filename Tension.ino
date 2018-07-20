#if TENSION

#include <Adafruit_PWMServoDriver.h>
#include "math.h"

#define OPTO_PIN          A0
#define OPTO_THRESHOLD    (300) // Opto value should range between 890-10
#define PWM_PIN           1
#define MIN_PWM_VALUE     (500)
#define SETTLING_TIME     (35)//35 //msed
#define MAX_SETTLING_TIME (50)//50 //ms

//#if (PWM_DEFINED == 0)
//#define PWM_DEFINED 1
//  Serial.println("Defined PWMController");
//  Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);
//#endif

void tensionSetup(){
  Serial.println("Setting up water tention measurement");
  #if PWMSTARTED == 0
    #define PWMSTARTED 1
      if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
        pwm.begin();
        xSemaphoreGive(xI2CSemaphore);
      }else{
        Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in tensionSetup\n\n");
        return 0;
      }
    //pwm.reset();
    //pwm.setPWMFreq(1600); //40-1600 Hz
  #endif

  //TaskHandle_t TaskHandle_Tension;
  xTaskCreate(
    vTaskTension,
    "tensionTask",
    670+100,
    NULL,
    1,
    &TaskHandle_Tension);
  
  Serial.print("Tension setup done");
  xSemaphoreGive(xTensionSemaphore);
}

uint16_t getWaterTension(uint8_t nr_of_measurements){
  uint32_t measurement_sum = 0;
  uint16_t measurements[nr_of_measurements] = {0};
  uint32_t pwmValue = 0;
  uint16_t delay_time = 0;
  uint8_t  print_resolution = 100;
  resetTensionMeasurement(600);
  vTaskDelay(5000);
  uint16_t last_measurement = getRoughTensionMeasurement(MIN_PWM_VALUE, 35, MAX_SETTLING_TIME*3);
  
  if (last_measurement == 0){
    Serial.println("\n************************\nTension measurement failed!");
    Serial.println("watertension may be too high or needle stuck\n");
    Serial.print("Opto before reset: ");Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
    setPWM(PWM_PIN, 0);
    vTaskDelay(50);
    Serial.print("Opto after reset:  ");Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
    return 0;
  }else{
    //Serial.print("Rough estimate is: ");Serial.println(last_measurement);
  }
  
  pwmValue = last_measurement;//resetTensionMeasurement(last_measurement);
  
  for (uint8_t loop_nr = 0; loop_nr < nr_of_measurements; loop_nr++)
  {
    pwmValue = resetTensionMeasurement(pwmValue);
    //*******************************
    // Last measurement -80 < 0??
    //*******************************
    
    //pwmValue = riseNeedle(MIN_PWM_VALUE, (last_measurement - 80), 30, MAX_SETTLING_TIME);
    if (last_measurement > 80){
      pwmValue = riseNeedle(MIN_PWM_VALUE, (last_measurement - 80), 30, MAX_SETTLING_TIME);
    }else{
      pwmValue = 0;
    }
    //Serial.print("  -pwmValue: ");Serial.println(pwmValue);
    
    while(analogRead(OPTO_PIN) > OPTO_THRESHOLD)
    {
      //Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
      if (pwmValue < (last_measurement-30)){
        delay_time = MAX_SETTLING_TIME;
      } else {
        delay_time = MAX_SETTLING_TIME*2;
      }
      pwmValue++;
      if (pwmValue >= 4096){
        Serial.println("\n\nWARNING pwmValue maxed out\n");
        Serial.print("Opto before reset: ");Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
        setPWM(PWM_PIN, 0);
        vTaskDelay(50);
        Serial.print("Opto after reset:  ");Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
        pwmValue = 4095;
        break;
      }
      setPWM(PWM_PIN,  pwmValue);
      vTaskDelay(delay_time);
    }
    last_measurement = pwmValue;
    //Serial.print("Opto: ");Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
    Serial.print("\n\tValue for loop #");Serial.print(loop_nr);Serial.print(": ");Serial.println(last_measurement);//Serial.println("");
    //Serial.print(last_measurement);Serial.print(";");
    measurement_sum += last_measurement;
    measurements[loop_nr] = last_measurement;
    
    //Serial.print("  -pwmValue: ");Serial.println(pwmValue);
    vTaskDelay(500); //Allow for other tasks to run
  }
  
  if (pwmValue>400){
    setPWM(PWM_PIN,  pwmValue-400);
  }
  //Serial.println(" -Calculating mean/median..");
  uint32_t returnValue = measurement_sum/nr_of_measurements;
  Serial.println("\n**********************************************");
  Serial.print("Water tention value: "); Serial.println(returnValue);
  Serial.print("\nMean: \t\t\t"); Serial.println(getMean(measurements, nr_of_measurements));
  uint16_t median = getMedian(measurements,nr_of_measurements);
  Serial.print("Median: \t\t"); Serial.println(median);
  Serial.print("Standard Deviation:\t"); Serial.println(getStandardDeviation(measurements, nr_of_measurements));
  Serial.println("\n");
  return (uint16_t)median;//returnValue;
  
}

float getMean(uint16_t data[], uint8_t arraySize){
  float mean = 0;
  for (uint8_t i = 0; i < arraySize; i++){
    mean += data[i];
  }
  mean = (mean/arraySize);
  return mean;
}

int sortCompare(const void * p1, const void * p2){
  return ( *(uint16_t*)p1 - *(uint16_t*)p2 );
}

uint16_t getMedian(uint16_t data[], uint8_t arraySize){
  qsort(data, arraySize,sizeof(uint16_t),sortCompare);
  uint16_t median = data[(arraySize/2)];
  return median;
}

float getStandardDeviation(uint16_t data[], uint8_t arraySize){
  float mean = getMean(data, arraySize);
  float sDeviation = 0;
  for (int i = 0; i < arraySize; i++){
    sDeviation += pow((data[i]-mean), 2);
  }
  return sqrt(sDeviation/arraySize);
}

uint16_t resetTensionMeasurement(uint16_t pwmValue){
  //Serial.println(" -resetTensionMeasurement");
//  if (pwmValue > 100){
//    pwmValue -= 100;
//  }
//  pwm.setPWM(PWM_PIN, 0, pwmValue);
  while (pwmValue > MIN_PWM_VALUE){
    if ((pwmValue > 1800)){
      pwmValue -= 300;
    }else if (pwmValue > 900){
      pwmValue -= 50;
      //Serial.print(" ");
    }else{
      pwmValue -= 10;
      //Serial.print("  ");
    }
    setPWM(PWM_PIN, pwmValue);
    
    vTaskDelay(MAX_SETTLING_TIME*4);
  }
  //Serial.println(pwmValue);
  vTaskDelay(3000);
  return pwmValue;
}

uint16_t riseNeedle(uint16_t startValue, uint16_t endValue, uint8_t resolution, uint8_t stepDelay){
  //Serial.println(" -riseNeedle");
  setPWM(PWM_PIN, startValue);
  vTaskDelay(stepDelay*2);
  while (startValue < (endValue-resolution)){
    startValue += resolution;
    setPWM(PWM_PIN, startValue);
    vTaskDelay(stepDelay);
  }
  return startValue;
}

uint16_t getRoughTensionMeasurement(uint16_t startValue, uint8_t resolution, uint8_t stepDelay){
  //Serial.println(" -getRoughTensionMeasurement");
  uint16_t pwmValue = startValue;
  setPWM(PWM_PIN, pwmValue);
  vTaskDelay(stepDelay * 2);
  while(analogRead(OPTO_PIN) > OPTO_THRESHOLD)
  {
    //Serial.print(analogRead(OPTO_PIN));Serial.print("/");Serial.println(OPTO_THRESHOLD);
    pwmValue += resolution;
    if (pwmValue >= 4096){
      return 0;
    }
    setPWM(PWM_PIN, pwmValue);
    vTaskDelay(stepDelay);
  }
  return (pwmValue-resolution);
}

void setPWM(uint8_t pin, uint16_t value){
  if (xSemaphoreTake(xI2CSemaphore, (TickType_t) 1000)){
    pwm.setPWM(pin, 0, value);
    xSemaphoreGive(xI2CSemaphore);
  }else{
    Serial.println("\n\n FAILED TO OBTAIN SEMAPHORE: in setPWM\n\n");
  }

}


void testPWM(uint8_t pin){
  // Loop pin to high
  for (uint16_t i=0; i<4096; i += 8) {
    pwm.setPWM(pin, 0, i );
    Serial.println(i);
    Serial.print("Opto value: "); Serial.println(analogRead(OPTO_PIN));
    vTaskDelay(10);
  }
  
  Serial.println("PWM Fully on");
  vTaskDelay(2000);
  //Loop pin to low
  for (uint16_t i=4096; i>8; i -= 8) {
    pwm.setPWM(pin, 0, i );
    Serial.println(i);
    vTaskDelay(10);
  }
  Serial.println("PWM Fully off");
}




static void vTaskTension(void* pvParameters){

  delay(1000);
  UBaseType_t uxHighWaterMark;
  uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
  uint16_t tension =  0;
  writeTension(getWaterTension(1));
  while(1){
    Serial.println("Started measurement of watertension");
    if (xSemaphoreTake(xTensionSemaphore, portMAX_DELAY)){
      tension = getWaterTension(8);
      xSemaphoreGive(xTensionSemaphore);
    }
    Serial.print("Watertension result: \t");Serial.println(tension);
    writeTension(tension);
    uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
    Serial.print("Tension measurement highwatermark: ");Serial.println(uxHighWaterMark);
    Serial.println("****************************************\n\n");
    vTaskDelay(15000);
  }
}



#endif
