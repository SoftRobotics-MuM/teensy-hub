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
*
* If that still fails with "No 'rosidl_typesupport_cpp' found" (CMake error
* inside builtin_interfaces/rosidl_typesupport_c): a system ROS 2 install
* sourced in your shell profile (e.g. `source /opt/ros/jazzy/setup.zsh` in
* .zshrc) leaks CMAKE_PREFIX_PATH/AMENT_PREFIX_PATH into this build, and it
* picks up the host ROS's C++ typesupport instead of the bundled C-only one
* this cross-compiled target needs. Rebuild with those stripped, e.g.:
*   env -u CMAKE_PREFIX_PATH -u AMENT_PREFIX_PATH -u ROS_DISTRO -u ROS_VERSION -u ROS_PYTHON_VERSION -u COLCON_PREFIX_PATH \
*     PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/opt/ros/jazzy' | paste -sd: -)" \
*     pio run
*/

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw/qos_profiles.h>
#include <soro_msgs/msg/servo_commands.h>
#include "servo_control.h"
#include "servo_cfg.h"
#include "sensor_cfg.h"
#include <soro_msgs/msg/sensor_array.h>
#if USE_NEW_SENSOR_PCB
  #include "sensor_pcb_v2.h"
#else
  #include <read_LSM6DSV16X.h>
  #include "LSM6DSV16X_regs.h"
  #include "MMC5603_regs.h"
#endif


#if !defined(ARDUINO_TEENSY41)
  #error This code only runs on Arduino Teensy41
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// parameters
// Note: The publish period times can be set to a value higher than 1000*1000*60 to effectively disable the respective publisher.
const uint32_t publish_period_time_servo_set_micros = 1000*1000*60; // period time in microseconds
const uint32_t publish_period_time_sensor_data_micros = 1000*1000*60; // period time in microseconds

// Note: Pin 13 (Teensy onboard LED) is also PIN_MOTOR_12's PWM output (see servo_cfg.h).
// Do not drive it with digitalWrite() - that would fight the hardware PWM and glitch motor 12.
// The LED simply reflects motor 12's PWM duty cycle as a side effect.

// micro-ROS configuration
rcl_publisher_t publisher;
rcl_publisher_t publisher_sensor_data;

rcl_subscription_t subscriber;
soro_msgs__msg__ServoCommands current_servo_pos_msg;
soro_msgs__msg__ServoCommands incoming_msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// QoS profile for the servo command channel (best-effort, depth 1): always act on the
// newest command rather than draining a backlog if the PC ever publishes faster than
// the Teensy consumes.
rmw_qos_profile_t qos_profile_servo_cmd = rmw_qos_profile_sensor_data;

// IMU (old hardware only - see sensor_cfg.h's USE_NEW_SENSOR_PCB)
#if !USE_NEW_SENSOR_PCB
const int NUM_IMU = 4; // Number of IMUs (old-hardware sites; maps 1:1 onto SensorArray.boards[0..3])
LSM6DSV16XSensor lsm6dsv16x_1(&Wire1, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_2(&Wire1, LSM6DSV16X_I2C_ADD_H);
LSM6DSV16XSensor lsm6dsv16x_3(&Wire2, LSM6DSV16X_I2C_ADD_L);
LSM6DSV16XSensor lsm6dsv16x_4(&Wire2, LSM6DSV16X_I2C_ADD_H);

LSM6DSV16XSensor imus[NUM_IMU] = {lsm6dsv16x_1, lsm6dsv16x_2, lsm6dsv16x_3, lsm6dsv16x_4};
#endif

#if defined(ARDUINO_TEENSY41)
void get_teensy_mac(uint8_t *mac) {
    for(uint8_t by=0; by<2; by++) mac[by]=(HW_OCOTP_MAC1 >> ((1-by)*8)) & 0xFF;
    for(uint8_t by=0; by<4; by++) mac[by+2]=(HW_OCOTP_MAC0 >> ((3-by)*8)) & 0xFF;
}
#endif

void error_loop(){
  while(1){
    Serial.println("micro-ROS error: halted in error_loop()");
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
 * @brief Read all sensor data (old I2C hardware or new sensor-pcb-v2 shared-SPI-bus
 * hardware, per sensor_cfg.h's USE_NEW_SENSOR_PCB) and publish it as one
 * soro_msgs::msg::SensorArray on "sensor_data" - see SensorArray.msg/SensorBoard.msg
 * for the wire layout. Rate-limited like the other publish_* functions.
 */
soro_msgs__msg__SensorArray sensor_data_msg;
void publish_sensor_data(uint32_t publish_interval_micros = 10000) {
  static uint32_t last_call_micros = 0;
  if (micros() - last_call_micros < publish_interval_micros) {
    return; // Limit the frequency of publishing to a period time of publish_interval_micros
  }
  last_call_micros = micros();

  sensor_data_msg.boards_populated = 0;

#if USE_NEW_SENSOR_PCB
  for (uint8_t addr = 0; addr < MAX_SENSOR_BOARDS; addr++) {
    if (!BOARD_ENABLED[addr]) {
      continue;
    }
    read_sensor_board(addr, sensor_data_msg.boards[addr]);
    sensor_data_msg.boards_populated |= (1 << addr);
  }
#else
  for (int i = 0; i < NUM_IMU; i++) {
    soro_msgs__msg__SensorBoard &board = sensor_data_msg.boards[i];
    board.status = 0;
    board.seq = 0;

    int32_t accel[3] = {0};   // mg
    int32_t angrate[3] = {0}; // mdps
    read_imu(imus[i], accel, angrate);
    board.imu1[0] = angrate[0] * MDPS_TO_RADPS;
    board.imu1[1] = angrate[1] * MDPS_TO_RADPS;
    board.imu1[2] = angrate[2] * MDPS_TO_RADPS;
    board.imu1[3] = accel[0] * MG_TO_MPS2;
    board.imu1[4] = accel[1] * MG_TO_MPS2;
    board.imu1[5] = accel[2] * MG_TO_MPS2;
    board.status |= soro_msgs__msg__SensorBoard__STATUS_IMU1_VALID;

    double mag_field_uT[3] = {0.0};
    read_magnetometer(imus[i], mag_field_uT);
    board.mag1[0] = mag_field_uT[0] * 1e-6f;
    board.mag1[1] = mag_field_uT[1] * 1e-6f;
    board.mag1[2] = mag_field_uT[2] * 1e-6f;
    board.status |= soro_msgs__msg__SensorBoard__STATUS_MAG1_VALID;

    sensor_data_msg.boards_populated |= (1 << i);
  }
#endif

  RCSOFTCHECK(rcl_publish(&publisher_sensor_data, &sensor_data_msg, NULL));
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

  Serial.begin(115200); // Initialize serial output for debug

  // Servo init
  for(uint8_t i = 0; i < NUM_MOTORS; i++) {
    motors[i].set_pulse_length(CENTER_SERVO_US);
  }

  // Sensor init
#if USE_NEW_SENSOR_PCB
  setup_sensor_pcb_v2();
#else
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
#endif

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

  qos_profile_servo_cmd.depth = 1; // keep only the latest servo command/position, never a backlog

  // create publishers
  // init_publishers();
  // publisher for current servo positions
  RCCHECK(rclc_publisher_init(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos_set",
    &qos_profile_servo_cmd));
  
  // publisher for all sensor data (see SensorArray.msg/SensorBoard.msg)
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_sensor_data,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, SensorArray),
    "sensor_data"));

  // Initialize subscriber on topic "subscribe_topic"
   RCCHECK(rclc_subscription_init(
     &subscriber,
     &node,
    // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, incoming_msg, Int32),
     ROSIDL_GET_MSG_TYPE_SUPPORT(soro_msgs, msg, ServoCommands),
    "servo_pos",
    &qos_profile_servo_cmd));
  
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
  // uint32_t publish_period_time_micros = 10000; // period time in microseconds
  if (publish_period_time_servo_set_micros < 1000*1000*60) {
      publish_current_servo_positions(publish_period_time_servo_set_micros);
  }

  if (publish_period_time_sensor_data_micros < 1000*1000*60) {
    publish_sensor_data(publish_period_time_sensor_data_micros);
  }

  // Spin executor to process incoming messages
  rclc_executor_spin_some(&executor, RCL_US_TO_NS(200));
}