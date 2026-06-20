#include "hal_microros.h"

#define FORCE_UNUSED(expr) do { rcl_ret_t _res = (expr); (void)_res; } while(0)

extern "C" {
  bool arduino_transport_open(struct uxrCustomTransport * transport) {
    return true;
  }

  bool arduino_transport_close(struct uxrCustomTransport * transport) {
    return true;
  }

  size_t arduino_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
    (void)errcode;
    return Serial1.write(buf, len);
  }

  size_t arduino_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
    (void)errcode;
    Serial1.setTimeout(timeout);
    return Serial1.readBytes((char *)buf, len);
  }
}

HAL_MicroROS::HAL_MicroROS() :
    state(WAITING_AGENT),
    uros_node(rcl_get_zero_initialized_node()),
    left_joint_pos(0.0),
    right_joint_pos(0.0),
    last_pub_time(0),
    last_ping_check(0) {
    memset(&uros_support, 0, sizeof(uros_support));
    memset(&uros_executor, 0, sizeof(uros_executor));
    memset(&uros_imu_publisher, 0, sizeof(uros_imu_publisher));
    memset(&uros_joint_state_publisher, 0, sizeof(uros_joint_state_publisher));

    joint_positions[0] = 0.0;
    joint_positions[1] = 0.0;
    joint_velocities[0] = 0.0;
    joint_velocities[1] = 0.0;
}

HAL_MicroROS::~HAL_MicroROS() {
    if (state == AGENT_CONNECTED) {
        destroy_node_and_publishers();
    }
}

void HAL_MicroROS::init() {
    // Initialize micro-ROS transport
    set_microros_transports();
}

bool HAL_MicroROS::init_node_and_publishers() {
  uros_allocator = rcl_get_default_allocator();

  // Create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, uros_allocator) != RCL_RET_OK) return false;

  // Initialize rclc support
  if (rclc_support_init_with_options(&uros_support, 0, NULL, &init_options, &uros_allocator) != RCL_RET_OK) {
    FORCE_UNUSED(rcl_init_options_fini(&init_options));
    return false;
  }
  FORCE_UNUSED(rcl_init_options_fini(&init_options));

  // Create node
  if (rclc_node_init_default(&uros_node, "abc_controller_node", "", &uros_support) != RCL_RET_OK) {
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  // Create publishers
  if (rclc_publisher_init_default(
      &uros_imu_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "/imu/data_raw") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  if (rclc_publisher_init_default(
      &uros_joint_state_publisher,
      &uros_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
      "/joint_states") != RCL_RET_OK) {
    FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  // Create executor
  if (rclc_executor_init(&uros_executor, &uros_support.context, 0, &uros_allocator) != RCL_RET_OK) {
    FORCE_UNUSED(rcl_publisher_fini(&uros_joint_state_publisher, &uros_node));
    FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
    FORCE_UNUSED(rcl_node_fini(&uros_node));
    FORCE_UNUSED(rclc_support_fini(&uros_support));
    return false;
  }

  // Initialize messages static parts
  uros_imu_msg.header.frame_id.data = (char*)"imu_link";
  uros_imu_msg.header.frame_id.size = strlen(uros_imu_msg.header.frame_id.data);
  uros_imu_msg.header.frame_id.capacity = uros_imu_msg.header.frame_id.size + 1;

  joint_names[0].data = (char*)"left_wheel";
  joint_names[0].size = strlen(joint_names[0].data);
  joint_names[0].capacity = joint_names[0].size + 1;
  joint_names[1].data = (char*)"right_wheel";
  joint_names[1].size = strlen(joint_names[1].data);
  joint_names[1].capacity = joint_names[1].size + 1;

  uros_joint_state_msg.name.capacity = 2;
  uros_joint_state_msg.name.size = 2;
  uros_joint_state_msg.name.data = joint_names;

  uros_joint_state_msg.position.capacity = 2;
  uros_joint_state_msg.position.size = 2;
  uros_joint_state_msg.position.data = joint_positions;

  uros_joint_state_msg.velocity.capacity = 2;
  uros_joint_state_msg.velocity.size = 2;
  uros_joint_state_msg.velocity.data = joint_velocities;

  uros_joint_state_msg.effort.capacity = 0;
  uros_joint_state_msg.effort.size = 0;
  uros_joint_state_msg.effort.data = NULL;

  return true;
}

void HAL_MicroROS::destroy_node_and_publishers() {
  FORCE_UNUSED(rclc_executor_fini(&uros_executor));
  FORCE_UNUSED(rcl_publisher_fini(&uros_joint_state_publisher, &uros_node));
  FORCE_UNUSED(rcl_publisher_fini(&uros_imu_publisher, &uros_node));
  FORCE_UNUSED(rcl_node_fini(&uros_node));
  FORCE_UNUSED(rclc_support_fini(&uros_support));
}

void HAL_MicroROS::update(float rpm_L, float rpm_R, const ahrs_data_t &ahrs_data) {
    switch (state) {
      case WAITING_AGENT: {
        uint32_t now_ms = millis();
        if (now_ms - last_ping_check > 1000) {
          last_ping_check = now_ms;
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            state = AGENT_AVAILABLE;
          }
        }
        break;
      }

      case AGENT_AVAILABLE:
        if (init_node_and_publishers()) {
          state = AGENT_CONNECTED;
        } else {
          state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED: {
        // Check if agent is still alive (every 2 seconds or so)
        uint32_t now_ms = millis();
        if (now_ms - last_ping_check > 2000) {
          last_ping_check = now_ms;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            state = AGENT_DISCONNECTED;
            break;
          }
        }

        // Get dt for joint position accumulation
        uint32_t current_time_us = micros();
        double dt = (last_pub_time > 0) ? (current_time_us - last_pub_time) * 1e-6 : 0.01;
        last_pub_time = current_time_us;

        // Convert RPM to joint velocities (rad/s)
        double left_vel = rpm_L * (2.0 * M_PI / 60.0);
        double right_vel = rpm_R * (2.0 * M_PI / 60.0);

        // Accumulate joint positions (rad)
        left_joint_pos += left_vel * dt;
        right_joint_pos += right_vel * dt;

        // Populate IMU Message
        uros_imu_msg.header.stamp.sec = ahrs_data.imu_data.timestamp * 1e-6;
        uros_imu_msg.header.stamp.nanosec = (ahrs_data.imu_data.timestamp % 1000000) * 1000;

        // Gyroscope is in rad/s in calibrated imu_data
        uros_imu_msg.angular_velocity.x = ahrs_data.imu_data.gyro[0];
        uros_imu_msg.angular_velocity.y = ahrs_data.imu_data.gyro[1];
        uros_imu_msg.angular_velocity.z = ahrs_data.imu_data.gyro[2];

        // Accelerometer is in m/s^2 in calibrated imu_data
        uros_imu_msg.linear_acceleration.x = ahrs_data.imu_data.accl[0];
        uros_imu_msg.linear_acceleration.y = ahrs_data.imu_data.accl[1];
        uros_imu_msg.linear_acceleration.z = ahrs_data.imu_data.accl[2];

        // Populate JointState Message
        uros_joint_state_msg.header.stamp.sec = uros_imu_msg.header.stamp.sec;
        uros_joint_state_msg.header.stamp.nanosec = uros_imu_msg.header.stamp.nanosec;

        uros_joint_state_msg.position.data[0] = left_joint_pos;
        uros_joint_state_msg.position.data[1] = right_joint_pos;

        uros_joint_state_msg.velocity.data[0] = left_vel;
        uros_joint_state_msg.velocity.data[1] = right_vel;

        // Publish IMU and JointState
        FORCE_UNUSED(rcl_publish(&uros_imu_publisher, &uros_imu_msg, NULL));
        FORCE_UNUSED(rcl_publish(&uros_joint_state_publisher, &uros_joint_state_msg, NULL));

        // Spin executor
        rclc_executor_spin_some(&uros_executor, RCL_MS_TO_NS(10));
        break;
      }

      case AGENT_DISCONNECTED:
        destroy_node_and_publishers();
        state = WAITING_AGENT;
        last_pub_time = 0;
        break;
    }
}
