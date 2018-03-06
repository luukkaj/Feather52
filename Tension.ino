#ifdef TENSION

#include <Adafruit_PWMServoDriver.h>
#include "math.h"

#define OPTO_PIN          A0
#define OPTO_THRESHOLD    200
#define PWM_PIN           1
#define MIN_PWM_VALUE     2500
#define SETTLING_TIME     35 //ms
#define MAX_SETTLING_TIME 50 //ms

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

void tensionSetup(){
  Serial.println("Setting up water tention measurement");
  pwm.begin();
  Serial.println("PWM begun");
  pwm.setPWMFreq(1600); //40-1600 Hz
  Serial.print("Tension setup done");
}

uint16_t getWaterTension(uint8_t nr_of_measurements){
  uint32_t measurement_sum = 0;
  uint16_t measurements[nr_of_measurements] = {0};
  uint16_t pwmValue = 0;
  uint16_t delay_time = 0;
  uint8_t  print_resolution = 100;
  uint16_t last_measurement = getRoughTensionMeasurement(MIN_PWM_VALUE, 10, MAX_SETTLING_TIME*2);
  
  Serial.print("Rough estimate is: ");Serial.println(last_measurement);
  
  pwmValue = resetTensionMeasurement(last_measurement);
  
  for (uint8_t loop_nr = 0; loop_nr < nr_of_measurements; loop_nr++)
  {
    pwmValue = riseNeedle(MIN_PWM_VALUE, (last_measurement - 50), 50, MAX_SETTLING_TIME);
    
    while(analogRead(OPTO_PIN) > OPTO_THRESHOLD)
    {
      //Serial.print("Opto value: ");Serial.println(analogRead(OPTO_PIN));
      /* Working slow implementation
      if (pwmValue < (last_measurement-100)){
        pwmValue += 10;
        pwm.setPWM(PWM_PIN, 0, pwmValue);
        delay_time = SETTLING_TIME;
      }else if (pwmValue < (last_measurement-50)){
        pwmValue += 5;
        delay_time = SETTLING_TIME;
        print_resolution = 20;
      }else if (pwmValue < (last_measurement-10)){
        pwmValue++;
        delay_time = MAX_SETTLING_TIME;
        print_resolution = 1;
      }else{ 
        pwmValue++;
        delay_time = MAX_SETTLING_TIME;
      }
      */
      if (pwmValue < (last_measurement-10)){
        delay_time = MAX_SETTLING_TIME;
      } else {
        delay_time = MAX_SETTLING_TIME*2;
      }
      last_measurement = pwmValue;
      pwmValue++;
      if (pwmValue >= 4096){
        Serial.println("WARNING pwmValue maxed out");
        break; 
      }
      pwm.setPWM(PWM_PIN, 0, pwmValue);
      delay(delay_time);
    }
    Serial.print("Value for loop #");Serial.print(loop_nr);Serial.print(": ");Serial.println(last_measurement);
    measurement_sum += last_measurement;
    measurements[loop_nr] = last_measurement;
    

    
    pwmValue = resetTensionMeasurement(pwmValue);
  }
  uint32_t returnValue = measurement_sum/nr_of_measurements;
  Serial.print("Water tention value: "); Serial.println(returnValue);
  Serial.print("\nMean: \t\t"); Serial.println(getMean(measurements, nr_of_measurements));
  uint16_t median = getMedian(measurements,nr_of_measurements);
  Serial.print("Median: \t\t"); Serial.println(median);
  Serial.print("Standard Deviation:\t"); Serial.println(getStandardDeviation(measurements, nr_of_measurements));
  return (uint16_t)returnValue;
  
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
  uint16_t median = data[(arraySize/2)-1];
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
  pwmValue -= 100;
  pwm.setPWM(PWM_PIN, 0, pwmValue);
  while (pwmValue > 100){
    pwmValue -= 100;
    pwm.setPWM(PWM_PIN, 0, pwmValue);
    delay(SETTLING_TIME);
  }
  return pwmValue;
}

uint16_t riseNeedle(uint16_t startValue, uint16_t endValue, uint8_t resolution, uint8_t stepDelay){
  pwm.setPWM(PWM_PIN, 0, startValue);
  delay(stepDelay*1.5);
  while (startValue < (endValue-resolution)){
    startValue += resolution;
    pwm.setPWM(PWM_PIN, 0, startValue);
    delay(stepDelay);
  }
  return startValue;
}

uint16_t getRoughTensionMeasurement(uint16_t startValue, uint8_t resolution, uint8_t stepDelay){
  uint16_t pwmValue = startValue;
  pwm.setPWM(PWM_PIN, 0, pwmValue);
  delay(stepDelay * 2);
  while(analogRead(OPTO_PIN) > OPTO_THRESHOLD)
  {
    pwmValue += resolution;
    pwm.setPWM(PWM_PIN, 0, pwmValue);
    delay(stepDelay);
  }
  return (pwmValue-resolution);
}

void setPWM(uint8_t pin, uint16_t value){
  pwm.setPWM(pin, 0, value);
  //Serial.print("PWM value: ");Serial.println(value);
}


void testPWM(uint8_t pin){
  // Loop pin to high
  for (uint16_t i=0; i<4096; i += 8) {
    pwm.setPWM(pin, 0, i );
    Serial.println(i);
    Serial.print("Opto value: "); Serial.println(analogRead(OPTO_PIN));
    delay(10);
  }
  
  Serial.println("PWM Fully on");
  delay(2000);
  //Loop pin to low
  for (uint16_t i=4096; i>8; i -= 8) {
    pwm.setPWM(pin, 0, i );
    Serial.println(i);
    delay(10);
  }
  Serial.println("PWM Fully off");
}



#endif
