#ifndef servo_cfg_h
#define servo_cfg_h

#include "servo_control.h"
#include "Arduino.h"
// pins
constexpr uint16_t PIN_MOTOR_1 = 2;
constexpr uint16_t PIN_MOTOR_2 = 3;
constexpr uint16_t PIN_MOTOR_3 = 4;
constexpr uint16_t PIN_MOTOR_4 = 5;
constexpr uint16_t PIN_MOTOR_5 = 6;
constexpr uint16_t PIN_MOTOR_6 = 7;
constexpr uint16_t PIN_MOTOR_7 = 8;
constexpr uint16_t PIN_MOTOR_8 = 9;
constexpr uint16_t PIN_MOTOR_9 = 10;
constexpr uint16_t PIN_MOTOR_10 = 11;
constexpr uint16_t PIN_MOTOR_11 = 12;
constexpr uint16_t PIN_MOTOR_12 = 13;

// servo parameters
constexpr uint16_t MIN_SERVO_US = 900;
constexpr uint16_t MAX_SERVO_US = 2100;
constexpr uint16_t CENTER_SERVO_US = 1500;
constexpr uint16_t PWM_RESOLUTION = 15;
constexpr uint16_t PWM_FREQUENCY = 300; // pwm frequecy [Hz] can be set to 300 for Chaservo DS20; Other servos can get destroyed by values other than 50
constexpr uint8_t NUM_MOTORS = 12;

// servo objects
// Note: defined (not just declared) here - include this header from main.cpp only.
ServoControl motors[NUM_MOTORS] = {
  ServoControl(PIN_MOTOR_1, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_2, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_3, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_4, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_5, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_6, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_7, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_8, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_9, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_10, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_11, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
  ServoControl(PIN_MOTOR_12, MIN_SERVO_US, MAX_SERVO_US, PWM_RESOLUTION, PWM_FREQUENCY),
};

#endif // servo_cfg_h
