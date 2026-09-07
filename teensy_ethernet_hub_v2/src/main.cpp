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

// parameters
// Note: The publish period times can be set to a value higher than 1000*1000*60 to effectively disable the respective publisher.
const uint32_t publish_period_time_servo_set_micros = 2000000*60; // period time in microseconds
const uint32_t publish_period_time_imu_micros = 10000; // period time in microseconds
const uint32_t publish_period_time_imu_rotation_micros = 10000; // period time in microseconds
const uint32_t publish_period_time_mag_micros = 10000; // period time in microseconds

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
geometry_msgs__msg__Accel accel_msg;
void publish_accel_data(const rcl_publisher_t *publisher_imu, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  int32_t accel[3] = {0};
  int32_t angrate[3] = {0};

  // geometry_msgs__msg__Accel accel_msg;

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
geometry_msgs__msg__Quaternion quat_msg;
geometry_msgs__msg__Vector3 gyroscope_bias_msg;
geometry_msgs__msg__Vector3 gravity_vector_msg;
void publish_imu_game_rotation_vector_data(const rcl_publisher_t *publisher_rotation, const rcl_publisher_t *publisher_gravity, const rcl_publisher_t *publisher_gyrobias, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  float quaternions[4] = {0};
  float gyroscope_bias[3] = {0};
  float gravity_vector[3] = {0};

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
geometry_msgs__msg__Vector3 mag_msg;
void publish_magnetometer_data(const rcl_publisher_t *publisher_imu, LSM6DSV16XSensor &imu, int imu_index = 0, uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros[NUM_IMU] = {0};
  if (micros() - last_call_micros[imu_index] < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros[imu_index] = micros();

  double mag_field[3] = {0.0};
  read_magnetometer(imu, mag_field);

  // geometry_msgs__msg__Vector3 mag_msg;

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

  IPAddress arduino_ip(192, 168, 1, 179);
  IPAddress agent_ip(192, 168, 1, 113);
  size_t domain_id = 72;
  set_microros_native_ethernet_transports(arduino_mac, arduino_ip, agent_ip, 8888);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  Serial.begin(115200); // Initialize serial output for debug
  delay(1000); // Wait for serial to initialize
  Serial.println("Starting micro-ROS Arduino Teensy Hub Node...");
  delay(1000); // Wait for serial to initialize


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
  // init_publishers();
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
   RCCHECK(rclc_subscription_init_best_effort(
     &subscriber,
     &node,
    // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, incoming_msg, Int32),
     ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos"));
  
  // Initialize executor with 1 handle (subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Add subscriber handle and callback to executor
  RCCHECK(rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &incoming_msg,
    &servo_command_subscription_callback,
    ON_NEW_DATA));

  Serial.println("Micro-ROS Arduino Teensy Hub Node started.");
}

void loop() {
  // Serial.println("Looping...");
  // publish messages
  // uint32_t publish_period_time_micros = 10000; // period time in microseconds
  if (publish_period_time_servo_set_micros < 1000*1000*60) {
      publish_current_servo_positions(publish_period_time_servo_set_micros);
  }

  if (publish_period_time_imu_micros < 1000*1000*60) {
    publish_accel_data(&publisher_IMU_1, imus[0], 0, publish_period_time_imu_micros);
    publish_accel_data(&publisher_IMU_2, imus[1], 1, publish_period_time_imu_micros);
    publish_accel_data(&publisher_IMU_3, imus[2], 2, publish_period_time_imu_micros);
    publish_accel_data(&publisher_IMU_4, imus[3], 3, publish_period_time_imu_micros);
  }

  if (publish_period_time_imu_rotation_micros < 1000*1000*60) {
    publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_1, &publisher_IMU_gravity_1, &publisher_IMU_gyrobias_1, imus[0], 0, publish_period_time_imu_rotation_micros);
    publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_2, &publisher_IMU_gravity_2, &publisher_IMU_gyrobias_2, imus[1], 1, publish_period_time_imu_rotation_micros);
    publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_3, &publisher_IMU_gravity_3, &publisher_IMU_gyrobias_3, imus[2], 2, publish_period_time_imu_rotation_micros);
    publish_imu_game_rotation_vector_data(&publisher_IMU_rotation_4, &publisher_IMU_gravity_4, &publisher_IMU_gyrobias_4, imus[3], 3, publish_period_time_imu_rotation_micros);
  }

  if (publish_period_time_mag_micros < 1000*1000*60) {
    publish_magnetometer_data(&publisher_mag_1, imus[0], 0, publish_period_time_mag_micros);
    publish_magnetometer_data(&publisher_mag_2, imus[1], 1, publish_period_time_mag_micros);
    publish_magnetometer_data(&publisher_mag_3, imus[2], 2, publish_period_time_mag_micros);
    publish_magnetometer_data(&publisher_mag_4, imus[3], 3, publish_period_time_mag_micros);
  }

  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}