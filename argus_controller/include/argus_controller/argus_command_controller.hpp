#ifndef ARGUS_CONTROLLER__ARGUS_COMMAND_CONTROLLER_HPP_
#define ARGUS_CONTROLLER__ARGUS_COMMAND_CONTROLLER_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace argus_controller
{

enum class State
{
  DISCONNECTED,
  IDLE,
  ARMING,
  TAKING_OFF,
  FLYING,
  LANDING
};

class ArgusCommandController : public rclcpp::Node
{
public:
  explicit ArgusCommandController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  void commandCallback(const std_msgs::msg::String::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void commandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void controlLoop();

  void publishOffboardControlMode();
  void publishTrajectorySetpoint();
  void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
  uint64_t nowMicros();

  double control_rate_{20.0};
  double movement_distance_{1.0};
  double lateral_distance_{1.0};
  double altitude_step_{0.5};
  double takeoff_altitude_{2.0};

  State state_{State::DISCONNECTED};
  bool armed_{false};
  uint8_t nav_state_{0};
  bool preflight_checks_pass_{false};
  bool failsafe_{false};
  bool gcs_connection_lost_{false};
  bool have_status_{false};
  bool have_odometry_{false};

  float pos_x_{0.0f};
  float pos_y_{0.0f};
  float pos_z_{0.0f};
  float yaw_{0.0f};

  float target_x_{0.0f};
  float target_y_{0.0f};
  float target_z_{0.0f};
  float hold_yaw_{0.0f};
  bool target_valid_{false};

  int setpoint_counter_{0};
  bool offboard_requested_{false};
  bool arm_requested_{false};
  int arm_attempts_{0};
  rclcpp::Time last_arm_request_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time arming_start_time_{0, 0, RCL_ROS_TIME};
  bool land_command_sent_{false};
};

}  // namespace argus_controller

#endif  // ARGUS_CONTROLLER__ARGUS_COMMAND_CONTROLLER_HPP_
