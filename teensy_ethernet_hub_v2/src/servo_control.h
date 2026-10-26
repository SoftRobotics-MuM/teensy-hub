#ifndef ServoControl_h
#define ServoControl_h
#include <stdint.h>

class ServoControl{
  public:
  ServoControl();
  ServoControl(uint8_t servo_pin, int min_servo_us, int max_servo_us);
  ServoControl(uint8_t servo_pin, int min_servo_us, int max_servo_us, int pwm_resolution, int pwm_frequency);

  bool set_pwm_ticks(long ticks);  // pwm ticks 
  bool set_pulse_length(int pulse_length);  // pulse length [us] or [1e-7s]; unit is automatically detected from value range: <5000 -> [us]; >=5000 -> [1e-7s]. Out-of-range values are rejected (returns false, setpoint unchanged).
  bool is_valid_pulse_length(int pulse_length);  // true if pulse_length (same units as set_pulse_length) lies within [min_servo_us, max_servo_us]
  
  // getter
  int get_min_servo_us() {return min_servo_us;}
  int get_max_servo_us() {return max_servo_us;}
  int get_pwm_resolution() {return pwm_resolution;}
  int get_pwm_frequency() {return pwm_frequency;}

  long get_period_time_us() {return period_time_us;}
  long get_min_servo_ticks() {return min_servo_ticks;}
  long get_max_servo_ticks() {return max_servo_ticks;}
  long get_center_servo_ticks() {return center_servo_ticks;}

  long get_servo_pulse_length_set() {return servo_pulse_length_set;} // Note: Die to discretization of the PWM signal, this value is typically not the actual pulse length set, but the pulse length sent to the servo.
  long get_servo_ticks_set() {return servo_ticks_set;}

  private:
  void init_servo();
  long to_tenth_us(int pulse_length);  // [us] or [1e-7s] (see set_pulse_length) -> [1e-7s]
  long map_(long _x, long _in_min, long _in_max, long _out_min, long _out_max);

  int min_servo_us = 900;
  int max_servo_us = 2100;
  int pwm_resolution = 15;
  int pwm_frequency = 50; // pwm frequecy [Hz] can be set to 300 for Chaservo DS20; Other servos can get destroyed by values other than 50

  long period_time_us;
  long min_servo_ticks;
  long max_servo_ticks;
  long center_servo_ticks;
  long servo_pulse_length_set;
  long servo_ticks_set;

  uint8_t servo_pin;

};



#endif