#include "servo_control.h"
#include "Arduino.h"

ServoControl::ServoControl(){
  ServoControl::init_servo();
}

ServoControl::ServoControl(uint8_t servo_pin, int min_servo_us, int max_servo_us){
  ServoControl::min_servo_us = min_servo_us;
  ServoControl::max_servo_us = max_servo_us;
    
  ServoControl::servo_pin = servo_pin;
  ServoControl::init_servo();
}

ServoControl::ServoControl(uint8_t servo_pin, int min_servo_us, int max_servo_us, int pwm_resolution, int pwm_frequency){
  ServoControl::min_servo_us = min_servo_us;
  ServoControl::max_servo_us = max_servo_us;
  ServoControl::pwm_resolution = pwm_resolution;
  ServoControl::pwm_frequency = pwm_frequency; // pwm frequecy [Hz] can be set to 300 for Chaservo DS20; Other servos can get destroyed by values other than 50
  
  ServoControl::servo_pin = servo_pin;
  ServoControl::init_servo();
}

void ServoControl::init_servo(){
  pinMode(ServoControl::servo_pin,OUTPUT);
  analogWriteResolution(ServoControl::pwm_resolution);
  analogWriteFrequency(ServoControl::servo_pin, ServoControl::pwm_frequency);

  ServoControl::period_time_us = ((long)1e6)/ServoControl::pwm_frequency;
  ServoControl::min_servo_ticks = ServoControl::map_(ServoControl::min_servo_us, 0, period_time_us, 0, pow(2,ServoControl::pwm_resolution)-1);  // map from pulse width [us] to PWm ticks []
  ServoControl::max_servo_ticks = ServoControl::map_(ServoControl::max_servo_us, 0, period_time_us, 0, pow(2,ServoControl::pwm_resolution)-1);  // map from pulse width [us] to PWm ticks []
  ServoControl::center_servo_ticks = (max_servo_ticks + min_servo_ticks)/(long)2;

  set_pulse_length(center_servo_ticks); // set to center position
}

bool ServoControl::set_pwm_ticks(long ticks){
  bool success = true;
  if (ticks < min_servo_ticks){
    ticks = min_servo_ticks;
    success = false;
  }
  if (ticks > max_servo_ticks){
    ticks = max_servo_ticks;
    success = false;
  }
  // write back the actually set servo position to servo object
  servo_ticks_set = ticks;
  servo_pulse_length_set = ServoControl::map_(ticks, min_servo_ticks, max_servo_ticks, 
    (long)ServoControl::min_servo_us*10, (long)ServoControl::max_servo_us*10);
  analogWrite(ServoControl::servo_pin, ticks);
  return(success);
}

bool ServoControl:: set_pulse_length(int pulse_length){
  if (pulse_length < 5000){
    pulse_length = pulse_length*10;
  }
  long pwm_ticks = ServoControl::map_((long)pulse_length, 0, period_time_us*10, 0, pow(2,ServoControl::pwm_resolution)-1);
  return(this->set_pwm_ticks(pwm_ticks));
}

long ServoControl::map_(long _x, long _in_min, long _in_max, long _out_min, long _out_max){
  return round(((double)_x - (double)_in_min) * ((double)_out_max - (double)_out_min) / ((double)_in_max - (double)_in_min) + (double)_out_min);
}