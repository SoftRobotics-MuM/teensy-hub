// https://docs.vulcanexus.org/en/latest/rst/microros_documentation/benchmarking/benchmarking.html?utm_source=chatgpt.com
// pio pkg install # Install dependencies
// pio run # Build the firmware
// pio run --target upload # Flash the firmware
// pio run --target clean_microros  # Clean library

/* How to run
* first start docker container with micro-ROS agent:
* docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
* then run this example on Arduino
* Ethernet addresses/ports:
* IP arduino: 192.168.1.177
* IP agent: 192.168.1.113
* Port: 8888
* 
* ros2 topic pub /teensy_hub/subscribe_topic std_msgs/msg/Int32 "{data: 10}"
* ros2 topic pub /teensy_hub/subscribe_topic soro_msgs/msg/ServoCommands "{servo_micros: [1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512]}"
*
* ros2 topic echo /teensy_hub/servo_pos_set
* ros2 topic pub /teensy_hub/servo_pos soro_msgs/msg/ServoCommands "{servo_micros: [1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1509, 1510, 1511, 1512]}"
* ros2 topic pub /teensy_hub/servo_pos soro_msgs/msg/ServoCommands "{servo_micros: [15100, 15200, 15300, 15400, 15500, 15600, 15700, 15800, 15900, 15100, 15110, 15120]}"
*
* ros2 run plotjuggler plotjuggler
*/

/*
* Fix ERROR:colcon.colcon_cmake.task.cmake.build:Could not find a shell extension for the command environment Failed <<< ament_cmake_python [0.02s, exited with code 1]
* clean everything:
* pio run --target clean
* rm -rf .pio/libdeps/
* rm -rf ~/.micro_ros_dev
*
* update extensions
* sudo apt update
* sudo apt install python3-colcon-common-extensions python3-pip
* pip3 install -U setuptools
*
*Force-rebuild micro-ROS dev deps
* cd .pio/libdeps/teensy41_eth/micro_ros_platformio
* rm -rf build/ install/ log/
* pio run
*/

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <soro_msgs/msg/servo_commands.h>
#include "servo_control.h"
#include "servo_cfg.h"
#include <geometry_msgs/msg/accel.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/quaternion.h>
#include <read_LSM6DSV16X.h>
#include "LSM6DSV16X_regs.h"
#include "MMC5603_regs.h"

#if !defined(ARDUINO_TEENSY41)
  #error This code only runs on Arduino Teensy41
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Pin configuration
constexpr uint16_t PIN_LED = 13;

// micro-ROS configuration
rcl_publisher_t publisher;
rcl_publisher_t publisher_IMU_1;
rcl_publisher_t publisher_IMU_2;
rcl_publisher_t publisher_IMU_3;
rcl_publisher_t publisher_IMU_4;

rcl_publisher_t publisher_mag_1;
rcl_publisher_t publisher_mag_2;
rcl_publisher_t publisher_mag_3;
rcl_publisher_t publisher_mag_4;

rcl_publisher_t publisher_IMU_gravity_1;
rcl_publisher_t publisher_IMU_gravity_2;
rcl_publisher_t publisher_IMU_gravity_3;
rcl_publisher_t publisher_IMU_gravity_4;

rcl_publisher_t publisher_IMU_gyrobias_1;
rcl_publisher_t publisher_IMU_gyrobias_2;
rcl_publisher_t publisher_IMU_gyrobias_3;
rcl_publisher_t publisher_IMU_gyrobias_4;

rcl_publisher_t publisher_IMU_rotation_1;
rcl_publisher_t publisher_IMU_rotation_2;
rcl_publisher_t publisher_IMU_rotation_3;
rcl_publisher_t publisher_IMU_rotation_4;

rcl_subscription_t subscriber;
soro_msgs__msg__ServoCommands current_servo_pos_msg;
soro_msgs__msg__ServoCommands incoming_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// IMU
const int NUM_IMU = 4; // Number of IMUs
LSM6DSV16XSensor lsm6dsv16x_1(&Wire1, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_2(&Wire1, LSM6DSV16X_I2C_ADD_H);
LSM6DSV16XSensor lsm6dsv16x_3(&Wire2, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_4(&Wire2, LSM6DSV16X_I2C_ADD_H);

LSM6DSV16XSensor imus[NUM_IMU] = {lsm6dsv16x_1, lsm6dsv16x_2, lsm6dsv16x_3, lsm6dsv16x_4};
//LSM6DSV16XSensor imus[NUM_IMU] = {lsm6dsv16x_1, lsm6dsv16x_2};

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

void error_loop(){
  while(1){
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    delay(100);
  }
}

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

/**
 * @brief Publish the servo positions currently set in the servo library to the "servo_pos_set" topic.
 * Note: This is the position sent to the servos via PWM, not neccessarily the actual position of the servos.
 * 
 * @param publish_interval_ms The interval in milliseconds at which the servo positions are published.
 * 
 * This function is called in the main loop to periodically publish the current servo positions.
 * It retrieves the current servo positions from the ServoControl objects and publishes them to the "servo_pos_set" topic.
 * The function uses a static variable to keep track of the last time it was called, ensuring that it only publishes at the specified interval.
 */
void publish_current_servo_positions(uint32_t publish_interval_micros = 1000000) {
  static uint32_t last_call_micros = 0;
  if (micros() - last_call_micros < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros = micros();

  for (int i = 0; i < NUM_MOTORS; i++) {
    current_servo_pos_msg.servo_micros[i] = motors[i].get_servo_pulse_length_set(); // Get the pulse length set for each servo
  }
  RCSOFTCHECK(rcl_publish(&publisher, &current_servo_pos_msg, NULL));
}


/**
 * @brief Publish accelerometer data from the specified IMU to the given publisher.
 * 
 * @param publisher_imu The publisher to which the accelerometer data will be published.
 * @param imu The IMU sensor from which to read the accelerometer and gyroscope data.
 * @param imu_index The index of the IMU in the array (default is 0). (needed for publish rate limiting)
 * 
 * This function reads the accelerometer and gyroscope data from the specified IMU,
 * fills a geometry_msgs__msg__Accel message with this data, and publishes it.
 */
// Note: The function limits the frequency of publishing to 100Hz by checking the time since the last call.
void publish_accel_data(const rcl_publisher_t *publisher_imu, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  int32_t accel[3] = {0};
  int32_t angrate[3] = {0};

  geometry_msgs__msg__Accel accel_msg;

  // Read accelerometer/gyroscope data
  read_imu(imu, accel, angrate);

  // Fill the accel_msg with the data
  accel_msg.linear.x = accel[0];
  accel_msg.linear.y = accel[1];
  accel_msg.linear.z = accel[2];

  accel_msg.angular.x = angrate[0];
  accel_msg.angular.y = angrate[1];
  accel_msg.angular.z = angrate[2];

  // Publish the accelerometer data
  rcl_publish(publisher_imu, &accel_msg, NULL);
}

/**
 * @brief Publish game rotation vector data from the specified IMU to the given publisher.
 * 
 * @param publisher_imu The publisher to which the game rotation vector data will be published.
 * @param imu The IMU sensor from which to read the game rotation vector data.
 * @param imu_index The index of the IMU in the array (default is 0). (needed for publish rate limiting)
 * 
 * This function reads the accelerometer and gyroscope data from the specified IMU,
 * fills a geometry_msgs__msg__Accel message with this data, and publishes it.
 */
// Note: The function limits the frequency of publishing to 100Hz by checking the time since the last call.
void publish_imu_game_rotation_vector_data(const rcl_publisher_t *publisher_rotation, const rcl_publisher_t *publisher_gravity, const rcl_publisher_t *publisher_gyrobias, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  float quaternions[4] = {0};
  float gyroscope_bias[3] = {0};
  float gravity_vector[3] = {0};

  geometry_msgs__msg__Quaternion quat_msg;
  geometry_msgs__msg__Vector3 gyroscope_bias_msg;
  geometry_msgs__msg__Vector3 gravity_vector_msg;

  // Read accelerometer/gyroscope data
  read_imu_game_rotation_vector(imu, quaternions, gyroscope_bias, gravity_vector);

  // Fill the accel_msg with the data
  quat_msg.x = quaternions[0];
  quat_msg.y = quaternions[1];
  quat_msg.z = quaternions[2];
  quat_msg.w = quaternions[3];  
  
  gyroscope_bias_msg.x = gyroscope_bias[0];
  gyroscope_bias_msg.y = gyroscope_bias[1];
  gyroscope_bias_msg.z = gyroscope_bias[2]; 

  gravity_vector_msg.x = gravity_vector[0];
  gravity_vector_msg.y = gravity_vector[1];
  gravity_vector_msg.z = gravity_vector[2];

  // Publish the game rotation vector data
  rcl_publish(publisher_rotation, &quat_msg, NULL);
  rcl_publish(publisher_gyrobias, &gyroscope_bias_msg, NULL);
  rcl_publish(publisher_gravity, &gravity_vector_msg, NULL);
}

/**
 * @brief Publish magnetometer data from the specified IMU (with connected slave magnetometer) to the given publisher.
 * 
 * @param publisher_imu The publisher to which the magnetometer data will be published.
 * @param imu The IMU sensor from which to read the magnetometer data.
 * @param imu_index The index of the IMU in the array (default is 0). (needed for publish rate limiting)
 * 
 * This function reads the accelerometer and gyroscope data from the specified IMU,
 * fills a geometry_msgs__msg__Accel message with this data, and publishes it.
 */
// Note: The function limits the frequency of publishing to 100Hz by checking the time since the last call.
void publish_magnetometer_data(const rcl_publisher_t *publisher_imu, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  double mag_field[3] = {0.0};
  read_magnetometer(imu, mag_field);

  // int32_t accel[3];
  // int32_t angrate[3];
  geometry_msgs__msg__Vector3 mag_msg;

  // Fill the mag_msg with the data
  mag_msg.x = mag_field[0];
  mag_msg.y = mag_field[1];
  mag_msg.z = mag_field[2];

  // Publish the magnetometer data
  rcl_publish(publisher_imu, &mag_msg, NULL);
}

// Subscriber callback function
void servo_command_subscription_callback(const void * msgin)
{
  const soro_msgs__msg__ServoCommands * msg_in = (const soro_msgs__msg__ServoCommands *)msgin;
  
  for (int i=0; i<NUM_MOTORS; i++){
    motors[i].set_pulse_length(msg_in->servo_micros[i]); // Set servo pulse length based on received message
  }
}

void setup() {
  // setup ethernet connection
   byte arduino_mac[] = { 0xAA, 0xBB, 0xCC, 0xEE, 0xDD, 0xFF };

  #if defined(ARDUINO_TEENSY41)
  get_teensy_mac(arduino_mac);
  #endif

  IPAddress arduino_ip(192, 168, 1, 177);
  IPAddress agent_ip(192, 168, 1, 113);
  size_t domain_id = 72;
  set_microros_native_ethernet_transports(arduino_mac, arduino_ip, agent_ip, 8888);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  Serial.begin(115200); // Initialize serial output for debug

  // Servo init
  for(uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].set_pulse_length(CENTER_SERVO_US);
  }

  // IMU init
  Wire.begin();
  Wire.setClock(1000000);

  Wire1.begin();
  Wire1.setClock(1000000);

  Wire2.begin();
  Wire2.setClock(1000000);

  for (int i = 0; i < NUM_IMU; i++) {
    setup_LSM6DSV16X(imus[i]);
    delay(100);
  }
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));

  // Set ROS domain id
  RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

  // Setup support structure.
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_ethernet_node", "teensy_hub", &support));

  // create publishers
  // publisher for current servo positions
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos_set"));
  
  // publishers for IMU data (acceleration + angular rate)
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
    "imu_data_1"));

  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_IMU_2,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
   "imu_data_2"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
    "imu_data_3"));
  
  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_IMU_4,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Accel),
   "imu_data_4"));

  // publishers for magnetometer
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_mag_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "mag_data_1"));
  
  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_mag_2,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
   "mag_data_2"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_mag_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "mag_data_3"));

  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_mag_4,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
   "mag_data_4"));

  // publishers for gravity vector
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gravity_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gravity_data_1"));
  
  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_IMU_gravity_2,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
   "imu_gravity_data_2"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gravity_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gravity_data_3"));
  
  RCCHECK(rclc_publisher_init_best_effort(
   &publisher_IMU_gravity_4,
   &node,
   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
   "imu_gravity_data_4"));

  // publishers for game rotation vector
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_rotation_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu_rotation_data_1"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_rotation_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu_rotation_data_2"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_rotation_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu_rotation_data_3"));

  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_rotation_4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Quaternion),
    "imu_rotation_data_4"));  

  // publishers for gyroscope bias
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gyrobias_1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gyrobias_data_1"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gyrobias_2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gyrobias_data_2")); 
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gyrobias_3,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gyrobias_data_3"));
  
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_IMU_gyrobias_4,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    "imu_gyrobias_data_4"));

  // Initialize subscriber on topic "subscribe_topic"
  // RCCHECK(rclc_subscription_init_best_effort(
  //   &subscriber,
  //   &node,
  //   // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, incoming_msg, Int32),
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
  //   "servo_pos"));
  
  // Initialize executor with 1 handle (subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Add subscriber handle and callback to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &incoming_msg,
    &servo_command_subscription_callback,
    ON_NEW_DATA));
}

void loop() {
  // publish messages
  uint32_t publish_period_time_micros = 10000; // period time in microseconds

  publish_period_time_micros = 1000000; // period time in microseconds
  publish_current_servo_positions(publish_period_time_micros);
  
  publish_period_time_micros = 10000; // period time in microseconds
  publish_accel_data(&publisher_IMU_1, imus[0], 0, publish_period_time_micros);
  publish_accel_data(&publisher_IMU_2, imus[1], 1, publish_period_time_micros);
  publish_accel_data(&publisher_IMU_3, imus[2], 2, publish_period_time_micros);
  publish_accel_data(&publisher_IMU_4, imus[3], 3, publish_period_time_micros);

  publish_period_time_micros = 10000; // period time in microseconds
  publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_1, &publisher_IMU_gravity_1, &publisher_IMU_gyrobias_1, imus[0], 0, publish_period_time_micros);
  publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_2, &publisher_IMU_gravity_2, &publisher_IMU_gyrobias_2, imus[1], 1, publish_period_time_micros);
  publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_3, &publisher_IMU_gravity_3, &publisher_IMU_gyrobias_3, imus[2], 2, publish_period_time_micros);
  publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_4, &publisher_IMU_gravity_4, &publisher_IMU_gyrobias_4, imus[3], 3, publish_period_time_micros);

  publish_period_time_micros = 10000; // period time in microseconds
  publish_magnetometer_data(&publisher_mag_1, imus[0], 0, publish_period_time_micros);
  publish_magnetometer_data(&publisher_mag_2, imus[1], 1, publish_period_time_micros);
  publish_magnetometer_data(&publisher_mag_3, imus[2], 2, publish_period_time_micros);
  publish_magnetometer_data(&publisher_mag_4, imus[3], 3, publish_period_time_micros);

  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}