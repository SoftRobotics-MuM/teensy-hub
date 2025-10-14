#include "read_LSM6DSV16X.h"
#include "LSM6DSV16X_regs.h"
#include "MMC5603_regs.h"

void setup_LSM6DSV16X(LSM6DSV16XSensor& sensor){
  sensor.begin();
  sensor.Set_X_Power_Mode(LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);
  sensor.Set_G_Power_Mode(LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);
  sensor.Enable_X();
  sensor.Enable_G();

  sensor.Set_X_FS(LSM6DSV16X_16g);  // LSM6DSV16X_2g | LSM6DSV16X_4g | LSM6DSV16X_8g | LSM6DSV16X_16g
  sensor.Set_G_FS(LSM6DSV16X_4000dps);  // LSM6DSV16X_125dps| LSM6DSV16X_250dps | LSM6DSV16X_500dps | LSM6DSV16X_1000dps | LSM6DSV16X_2000dps | LSM6DSV16X_4000dps

  sensor.FIFO_Set_Mode(LSM6DSV16X_BYPASS_MODE); // Don't store IMU data in FIFO, otherwise reading the FIFO takes too long

  sensor.Set_X_ODR(960.0f, LSM6DSV16X_ACC_HIGH_PERFORMANCE_MODE);  //[Hz] 7.5f | 15.0f | 30.0f | 60.0f | 120.0f | 240.0f | 480.0f | 960.0f | 1920.0f | 3840.0f | 7680.0f 
  sensor.Set_G_ODR(960.0f, LSM6DSV16X_GYRO_HIGH_PERFORMANCE_MODE);  //[Hz] 7.5f | 15.0f | 30.0f | 60.0f | 120.0f | 240.0f | 480.0f | 960.0f | 1920.0f | 3840.0f | 7680.0f 
  sensor.Set_SFLP_ODR(480.0f);  //[Hz] 15.0f | 30.0f | 60.0f | 120.0f | 240.0f | 480.0f
  sensor.Enable_Rotation_Vector();
  sensor.Enable_Gravity_Vector();
  sensor.Enable_Gyroscope_Bias();
  sensor.FIFO_Set_Mode(LSM6DSV16X_STREAM_MODE);

  delay(10);

  // setup magnetometer
  sensor.Write_Reg(REG_LSM6DSV16X_IF_CFG, 0b01000000); // Enables I2C pull ups
  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b01000000); // Enables access to the sensor hub registers

  // set sample rate
  // REG_MMC5603_ODR: [ODR[7:0]] set sample rate to 255 (or 1000 if also (hpower=1 and no automatic set/reset))
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_ODR); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 255);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // calculate counter target value
  // REG_MMC5603_INT_CTR_0: [Cmm_freq_en = true | Auto_st_en = false | Auto_SR_en = false | Do Reset = false | Do Set = false | Start_MDT = reserved | Take_meas_T = false | Take_meas_M = false]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = 0x30|write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b10000000);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // enable automatic set/reset
  // REG_MMC5603_INT_CTR_0: [Cmm_freq_en = false | Auto_st_en = false | Auto_SR_en = true | Do Reset = false | Do Set = false | Start_MDT = reserved | Take_meas_T = false | Take_meas_M = false]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b00100000);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // set bandwidth (1.2 ms); limitied by sample rate, automatic set/reset
  // REG_MMC5603_INT_CTR_1: [Sw_reset = false | St_enm = false | St_enp = false | Z-inhibit = z-activr | Y-inhibit = y-active | X-inhibit = x-active | BW[1:0] = 1.2 ms]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_1); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b00000011);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // enable continuous mode
  // REG_MMC5603_INT_CTR_2: [hpower = true (makes 1000 Hz possible if automatic set/reset turned off) | reserved | reserved | Cmm_en = continuous mode | En_prd_se = periodical set enabled | Prd_set[2:0] = set@1/25/sample]
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000100); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 1 sensor]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 0); // slave address: [7-bit address = MMC5603_I2C_ADDR | write]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_INT_CTR_2); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_DATAWRITE_SLV0, 0b10011001);
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10000000); // slave config: [SHUB_ODR_[2:0] = 120 Hz | 0 | BATCH_EXT_SENS_0_EN = 0 | Slave0_numop[2:0] = 0]
  delay(100);

  // cofigure which registers to read
  // Note: as there are too many registers this has to be split "virtually" to two external devices (0 and 1)
  sensor.Write_Reg(REG_LSM6DSV16X_MASTER_CONFIG, 0b00000101); // Master config register: [RST_MASTER_REGS = false | WRITE_ONCE = false | START_CONFIG = acc/gyro trigger | PASS_THROUGH_MODE = disabled | MASTER_ON = true | AUX_SENS_ON[1:0] = 2 sensors]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_ADD, (MMC5603_I2C_ADDR << 1) | 1); // slave0 address: [7-bit address = MMC5603_I2C_ADDR | read]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_ADD, (MMC5603_I2C_ADDR << 1) | 1); // slave1 address: [7-bit address = MMC5603_I2C_ADDR | read]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_SUBADD, REG_MMC5603_X_OUT_0); // slave0 register address
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_SUBADD, REG_MMC5603_X_OUT_2); // slave1 register address
  sensor.Write_Reg(REG_LSM6DSV16X_SLV0_CONFIG, 0b10100110); // SLV0_CONFIG: [SHUB_ODR_[2:0] = 240 Hz | BATCH_EXT_SENS_0_EN = FIFI batching disabled | Slave0_numop[2:0] = read 6 registers]
  sensor.Write_Reg(REG_LSM6DSV16X_SLV1_CONFIG, 0b00010011); // SLV1_CONFIG: [reserved | reserved | reserved | reserved | BATCH_EXT_SENS_0_EN = FIFI batching disabled | Slave1_numop[2:0] = read 3 registers]
  delay(100);

  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b00000000); // Disables access to the sensor hub registers
}

void read_magnetometer(LSM6DSV16XSensor& sensor, double* mag_field){
  // read sensor
  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b01000000); // Enables access to the sensor hub
  uint8_t data_ext[9];
  int j=0;
  for (int reg_address=REG_LSM6DSV16X_SENSOR_HUB_1; reg_address<REG_LSM6DSV16X_SENSOR_HUB_1+9; reg_address++){
    sensor.Read_Reg(reg_address, data_ext+j);
  j++;
  }
  sensor.Write_Reg(REG_LSM6DSV16X_FUNC_CFG_ACCESS, 0b00000000); // Diasbles access to the sensor hub

  // calculate
  int32_t x_raw;
  int32_t y_raw;
  int32_t z_raw;

  x_raw = (uint32_t)data_ext[0] << 12 | (uint32_t)data_ext[1] << 4 |
      (uint32_t)data_ext[6] >> 4;
  y_raw = (uint32_t)data_ext[2] << 12 | (uint32_t)data_ext[3] << 4 |
      (uint32_t)data_ext[7] >> 4;
  z_raw = (uint32_t)data_ext[4] << 12 | (uint32_t)data_ext[5] << 4 |
      (uint32_t)data_ext[8] >> 4;
  
  // fix center offsets
  x_raw -= (uint32_t)1 << 19;
  y_raw -= (uint32_t)1 << 19;
  z_raw -= (uint32_t)1 << 19;

  double x = (double)x_raw*0.00625; // scale to uT by LSB in datasheet
  double y = (double)y_raw*0.00625;
  double z = (double)z_raw*0.00625;

  mag_field[0] = x;
  mag_field[1] = y;
  mag_field[2] = z;
}

void read_imu(LSM6DSV16XSensor& sensor, int32_t* accel, int32_t* angrate){

  // read linear and angular acceleration
  sensor.Get_X_Axes(accel); // mg
  sensor.Get_G_Axes(angrate);
}

void read_imu_game_rotation_vector(LSM6DSV16XSensor& sensor, float* quaternions, float* gyroscope_bias, float* gravity_vector){

  uint16_t fifo_samples;
  uint8_t tag = 0;
  // Check the number of samples inside FIFO
  if (sensor.FIFO_Get_Num_Samples(&fifo_samples) != LSM6DSV16X_OK) {
    Serial.println("LSM6DSV16X Sensor failed to get number of samples inside FIFO");
  }
  else if (fifo_samples > 0) { // Read the FIFO if there is one stored sample
    for (int i = 0; i < fifo_samples; i++) {
      sensor.FIFO_Get_Tag(&tag);
      if (tag == TAG_LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR) {
        sensor.FIFO_Get_Rotation_Vector(&quaternions[0]);
      }
      else if (tag == TAG_LSM6DSV16X_SFLP_GYROSCOPE_BIAS) {
        sensor.FIFO_Get_Gyroscope_Bias(&gyroscope_bias[0]);
      }
      else if (tag == TAG_LSM6DSV16X_SFLP_GRAVITY_VECTOR) {
        sensor.FIFO_Get_Gravity_Vector(&gravity_vector[0]);
      }
    }
  }
}