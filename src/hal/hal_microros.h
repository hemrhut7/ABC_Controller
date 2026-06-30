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

class HAL_MicroROS {
public:
    HAL_MicroROS();
    ~HAL_MicroROS();

    void init();
    void update(float rpm_L, float rpm_R, const ahrs_data_t &ahrs_data, const mag_data_t &mag_data, const baro_data_t &baro_data);

private:
    enum States {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    } state;

    bool init_node_and_publishers();
    void destroy_node_and_publishers();

    // micro-ROS variables
    rcl_node_t uros_node;
    rcl_allocator_t uros_allocator;
    rclc_support_t uros_support;
    rclc_executor_t uros_executor;

    rcl_publisher_t uros_imu_publisher;
    rcl_publisher_t uros_joint_state_publisher;
    rcl_publisher_t uros_mag_publisher;
    rcl_publisher_t uros_baro_publisher;

    sensor_msgs__msg__Imu uros_imu_msg;
    sensor_msgs__msg__JointState uros_joint_state_msg;
    sensor_msgs__msg__MagneticField uros_mag_msg;
    sensor_msgs__msg__FluidPressure uros_baro_msg;

    double left_joint_pos;
    double right_joint_pos;
    uint32_t last_pub_time;
    uint32_t last_ping_check;
    uint32_t last_sync_time;

    rosidl_runtime_c__String joint_names[2];
    double joint_positions[2];
    double joint_velocities[2];
};
