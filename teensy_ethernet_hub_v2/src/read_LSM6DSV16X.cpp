#include "read_LSM6DSV16X.h"
/*#include <read_LSM6DSV16X.h>


void setup_LSM6DSV16X() {
  // Initialize i2c.
  Wire.begin();

  lsm6dsv16x_1.begin();
  lsm6dsv16x_1.Enable_X();
  lsm6dsv16x_1.Enable_G();
}

void read_LSM6DSV16X() {
  int16_t accel[3];
  int16_t gyro[3];
  uint8_t orientation_xl, orientation_xh, orientation_yl, orientation_yh, orientation_zl, orientation_zh;

  // Read accelerometer data
  lsm6dsv16x_1.Get_X_AxesRaw(accel);

  // Read gyroscope data
  lsm6dsv16x_1.Get_G_AxesRaw(gyro);
  // Process or publish the data as needed
}
  */
// LSM6DSV16X_Read::LSM6DSV16X_Read(TwoWire *wire, uint8_t address)
// {
//     lsm6dsv16x = LSM6DSV16XSensor(wire, address);
// }