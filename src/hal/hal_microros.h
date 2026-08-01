#pragma once

#include "hal_type_define.h"
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#define UROS_DATA_QUEUE_DEPTH 25

class HAL_MicroROS {
public:
    HAL_MicroROS();
    ~HAL_MicroROS();

    void init();
    void push_data(const system_state_t &packet);
    void update();

    bool is_connected() const { return state == AGENT_CONNECTED; }
    bool is_transmitting() const { return state == AGENT_CONNECTED; }

private:
    QueueHandle_t data_queue = nullptr;

    enum States {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state;

    bool init_node_and_publishers();
    void destroy_node_and_publishers();
    void process_system_state();

    // micro-ROS variables
    rcl_node_t uros_node;
    rcl_allocator_t uros_allocator;
    rclc_support_t uros_support;
    rclc_executor_t uros_executor;

    rcl_publisher_t uros_imu_publisher;
    rcl_publisher_t uros_joint_state_publisher;
    rcl_publisher_t uros_mag_publisher;
    rcl_publisher_t uros_baro_publisher;
    rcl_publisher_t uros_battery_publisher;
    rcl_publisher_t uros_temp_publisher;
    rcl_publisher_t uros_mode_publisher;
    rcl_publisher_t uros_delay_publisher;
    rcl_publisher_t uros_pid_target_publisher;

    rcl_subscription_t uros_cmd_vel_subscriber;
    geometry_msgs__msg__Twist uros_cmd_vel_msg;
    rcl_subscription_t uros_cmd_mode_subscriber;
    std_msgs__msg__Int32 uros_cmd_mode_msg;

    sensor_msgs__msg__Imu uros_imu_msg;
    sensor_msgs__msg__JointState uros_joint_state_msg;
    sensor_msgs__msg__MagneticField uros_mag_msg;
    sensor_msgs__msg__FluidPressure uros_baro_msg;
    sensor_msgs__msg__BatteryState uros_battery_msg;
    sensor_msgs__msg__Temperature uros_temp_msg;
    std_msgs__msg__Int32 uros_mode_msg;
    std_msgs__msg__Int32 uros_delay_msg;
    std_msgs__msg__Float32MultiArray uros_pid_target_msg;

    double left_joint_pos;
    double right_joint_pos;
    uint32_t last_pub_time;
    uint32_t last_ping_check;
    uint32_t last_sync_time;

    // Tracking variables for new data publication
    uint64_t last_imu_timestamp;
    uint32_t last_joint_pub_ms;
    uint64_t last_mag_timestamp;
    uint64_t last_baro_timestamp;
    uint64_t last_temp_timestamp;
    uint32_t last_battery_pub_ms;
    float last_battery_v;
    int last_mode;
    uint32_t last_delay_count;
    float last_pid_target[6];

    rosidl_runtime_c__String joint_names[2];
    double joint_positions[2];
    double joint_velocities[2];
    double joint_efforts[2];
    float pid_target_data[6];
};

