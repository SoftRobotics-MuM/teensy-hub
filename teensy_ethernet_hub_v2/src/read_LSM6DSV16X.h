#ifndef read_LSM6DSV16X_h
#define read_LSM6DSV16X_h

#include <LSM6DSV16XSensor.h>

void setup_LSM6DSV16X(LSM6DSV16XSensor& sensor);

void read_imu(LSM6DSV16XSensor& sensor, int32_t* accel, int32_t* angrate);
void read_imu_game_rotation_vector(LSM6DSV16XSensor& sensor, float* quaternions, float* gyroscope_bias, float* gravity_vector);
void read_magnetometer(LSM6DSV16XSensor& sensor, double* mag_field);


#endif