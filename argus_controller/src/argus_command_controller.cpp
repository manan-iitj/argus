#include "argus_controller/argus_command_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>

namespace argus_controller
{

namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// ============================================================================
// ARGUS flight tuning constants -- edit these and rebuild to change behaviour.
// All distances are in metres. Directions are relative to the drone's heading.
// ============================================================================

// Distance moved per FRONT / BACK command (along the drone's heading, x).
constexpr double kForwardStep = 1.0;

// Distance moved per LEFT / RIGHT command (perpendicular to heading, y).
constexpr double kLateralStep = 1.0;

// Altitude change per UP / DOWN command (z).
constexpr double kAltitudeStep = 0.5;

// Height above the launch point to climb to on LAUNCH.
constexpr double kTakeoffAltitude = 2.0;

// Takeoff is considered complete when within this distance of the target height.
constexpr double kTakeoffTolerance = 0.15;

// Offboard setpoint / control loop rate (Hz). PX4 requires > 2 Hz.
constexpr double kControlRate = 20.0;

// Seconds between ARM retries, and total time before a launch is aborted.
constexpr double kArmRetryInterval = 1.0;
constexpr double kArmTimeout = 15.0;
}  // namespace

ArgusCommandController::ArgusCommandController(const rclcpp::NodeOptions & options)
: Node("argus_command_controller", options)
{
  // The constants above are the defaults; ROS parameters of the same name
  // (if supplied) override them at runtime.
  control_rate_ = declare_parameter<double>("control_rate", kControlRate);
  movement_distance_ = declare_parameter<double>("movement_distance", kForwardStep);
  lateral_distance_ = declare_parameter<double>("lateral_distance", kLateralStep);
  altitude_step_ = declare_parameter<double>("altitude_step", kAltitudeStep);
  takeoff_altitude_ = declare_parameter<double>("takeoff_altitude", kTakeoffAltitude);

  auto qos = rclcpp::QoS(10);
  qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  command_sub_ = create_subscription<std_msgs::msg::String>(
    "/argus/command", 10,
    std::bind(&ArgusCommandController::commandCallback, this, std::placeholders::_1));

  vehicle_status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    "/fmu/out/vehicle_status_v4", qos,
    std::bind(&ArgusCommandController::vehicleStatusCallback, this, std::placeholders::_1));

  vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    "/fmu/out/vehicle_odometry", qos,
    std::bind(&ArgusCommandController::vehicleOdometryCallback, this, std::placeholders::_1));

  command_ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
    "/fmu/out/vehicle_command_ack_v1", qos,
    std::bind(&ArgusCommandController::commandAckCallback, this, std::placeholders::_1));

  offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    "/fmu/in/offboard_control_mode", qos);

  trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    "/fmu/in/trajectory_setpoint", qos);

  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", qos);

  const auto period = std::chrono::duration<double>(1.0 / control_rate_);
  control_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&ArgusCommandController::controlLoop, this));

  RCLCPP_INFO(get_logger(), "[ARGUS] Controller started");
}

void ArgusCommandController::commandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string cmd = msg->data;

  std::transform(cmd.begin(), cmd.end(), cmd.begin(),
    [](unsigned char c) { return std::toupper(c); });

  RCLCPP_INFO(get_logger(), "[ARGUS] Command: %s", cmd.c_str());

  if (cmd == "LAUNCH") {
    if (!have_status_ || !have_odometry_) {
      RCLCPP_WARN(get_logger(), "[ARGUS] PX4 feedback unavailable");
      return;
    }

    if (!armed_ && state_ == State::IDLE) {
      target_x_ = pos_x_;
      target_y_ = pos_y_;
      target_z_ = pos_z_ - takeoff_altitude_;
      hold_yaw_ = yaw_;
      target_valid_ = true;
      setpoint_counter_ = 0;
      offboard_requested_ = false;
      arm_requested_ = false;
      arm_attempts_ = 0;
      arming_start_time_ = now();
      state_ = State::ARMING;
      RCLCPP_INFO(get_logger(), "[ARGUS] Launch sequence started");
      if (!preflight_checks_pass_) {
        RCLCPP_WARN(get_logger(),
          "[ARGUS] PX4 reports preflight checks NOT passing - arming will likely be rejected. "
          "Check the PX4 console (pxh>) / QGroundControl for the 'Preflight Fail' reason.");
      }
    }

    return;
  }

  if (cmd == "LAND") {
    if (state_ == State::FLYING || state_ == State::ARMING || state_ == State::TAKING_OFF) {
      state_ = State::LANDING;
      land_command_sent_ = false;
      RCLCPP_INFO(get_logger(), "[ARGUS] Landing");
    }

    return;
  }

  if (cmd == "INSPECT") {
    if (state_ == State::FLYING) {
      RCLCPP_INFO(get_logger(), "[ARGUS] Inspection command received");
    }

    return;
  }

  if (state_ != State::FLYING || !have_odometry_) {
    return;
  }

  const double c = std::cos(yaw_);
  const double s = std::sin(yaw_);

  if (!target_valid_) {
    target_x_ = pos_x_;
    target_y_ = pos_y_;
    target_z_ = pos_z_;
    target_valid_ = true;
  }

  if (cmd == "FRONT") {
    target_x_ += movement_distance_ * c;
    target_y_ += movement_distance_ * s;
  } else if (cmd == "BACK") {
    target_x_ -= movement_distance_ * c;
    target_y_ -= movement_distance_ * s;
  } else if (cmd == "RIGHT") {
    target_x_ -= lateral_distance_ * s;
    target_y_ += lateral_distance_ * c;
  } else if (cmd == "LEFT") {
    target_x_ += lateral_distance_ * s;
    target_y_ -= lateral_distance_ * c;
  } else if (cmd == "UP") {
    target_z_ -= altitude_step_;
  } else if (cmd == "DOWN") {
    target_z_ += altitude_step_;
  } else {
    return;
  }

  RCLCPP_INFO(get_logger(), "[ARGUS] Target: %.2f %.2f %.2f", target_x_, target_y_, target_z_);
}

void ArgusCommandController::vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  have_status_ = true;

  armed_ = msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  nav_state_ = msg->nav_state;

  if (msg->pre_flight_checks_pass != preflight_checks_pass_) {
    RCLCPP_INFO(get_logger(), "[ARGUS] Preflight checks: %s",
      msg->pre_flight_checks_pass ? "PASS" : "FAIL");
  }
  preflight_checks_pass_ = msg->pre_flight_checks_pass;
  failsafe_ = msg->failsafe;
  gcs_connection_lost_ = msg->gcs_connection_lost;
}

void ArgusCommandController::vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  have_odometry_ = true;

  pos_x_ = msg->position[0];
  pos_y_ = msg->position[1];
  pos_z_ = msg->position[2];

  const double w = msg->q[0];
  const double x = msg->q[1];
  const double y = msg->q[2];
  const double z = msg->q[3];

  yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

void ArgusCommandController::commandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "[ARGUS] ACK command=%u result=%u", msg->command, msg->result);

  if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM &&
    msg->result != px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED)
  {
    RCLCPP_WARN(get_logger(),
      "[ARGUS] ARM rejected by PX4 (result=%u). preflight_checks_pass=%s failsafe=%s "
      "gcs_connection_lost=%s. See PX4 console for the exact 'Arming denied' reason.",
      msg->result,
      preflight_checks_pass_ ? "true" : "false",
      failsafe_ ? "true" : "false",
      gcs_connection_lost_ ? "true" : "false");
  }
}

void ArgusCommandController::controlLoop()
{
  if (!have_status_ || !have_odometry_) {
    return;
  }

  if (state_ == State::DISCONNECTED) {
    state_ = State::IDLE;
    RCLCPP_INFO(get_logger(), "[ARGUS] PX4 connected");
  }

  if (state_ == State::ARMING) {
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    setpoint_counter_++;

    if (setpoint_counter_ >= 10 && !offboard_requested_) {
      RCLCPP_INFO(get_logger(), "[ARGUS] Requesting OFFBOARD");
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0f, 6.0f);
      offboard_requested_ = true;
    }

    // Retry ARM once per second while in OFFBOARD and not yet armed
    // (PX4 may temporarily reject arming, e.g. right after the mode switch
    // or while estimator/preflight checks are still settling).
    if (offboard_requested_ && !armed_ &&
      nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD &&
      (!arm_requested_ || (now() - last_arm_request_time_).seconds() >= kArmRetryInterval))
    {
      arm_attempts_++;
      RCLCPP_INFO(get_logger(), "[ARGUS] OFFBOARD active -> ARM (attempt %d)", arm_attempts_);
      publishVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0f, 0.0f);
      arm_requested_ = true;
      last_arm_request_time_ = now();
    }

    // Give up after a timeout so a new LAUNCH can be issued.
    if (!armed_ && (now() - arming_start_time_).seconds() > kArmTimeout) {
      RCLCPP_ERROR(get_logger(),
        "[ARGUS] Launch aborted: vehicle did not arm within %.0f s (attempts=%d). "
        "Fix the PX4 arming/preflight issue and send LAUNCH again.", kArmTimeout, arm_attempts_);
      state_ = State::IDLE;
      target_valid_ = false;
      return;
    }

    if (armed_ && nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      state_ = State::TAKING_OFF;
      RCLCPP_INFO(get_logger(), "[ARGUS] Armed -> TAKEOFF");
    }

    return;
  }

  if (state_ == State::TAKING_OFF) {
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    if (pos_z_ <= target_z_ + kTakeoffTolerance) {
      state_ = State::FLYING;
      RCLCPP_INFO(get_logger(), "[ARGUS] Takeoff complete");
    }

    return;
  }

  if (state_ == State::FLYING) {
    publishOffboardControlMode();
    publishTrajectorySetpoint();

    return;
  }

  if (state_ == State::LANDING) {
    if (!land_command_sent_) {
      RCLCPP_INFO(get_logger(), "[ARGUS] Sending LAND");
      publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
      land_command_sent_ = true;
    }

    publishOffboardControlMode();
    publishTrajectorySetpoint();

    if (!armed_) {
      state_ = State::IDLE;
      land_command_sent_ = false;
      target_valid_ = false;
      RCLCPP_INFO(get_logger(), "[ARGUS] Landed");
    }

    return;
  }
}

void ArgusCommandController::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};

  msg.timestamp = nowMicros();

  msg.position = true;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;

  offboard_mode_pub_->publish(msg);
}

void ArgusCommandController::publishTrajectorySetpoint()
{
  px4_msgs::msg::TrajectorySetpoint msg{};

  msg.timestamp = nowMicros();

  msg.position[0] = target_valid_ ? target_x_ : pos_x_;
  msg.position[1] = target_valid_ ? target_y_ : pos_y_;
  msg.position[2] = target_valid_ ? target_z_ : pos_z_;

  msg.yaw = target_valid_ ? hold_yaw_ : yaw_;

  msg.velocity[0] = kNaN;
  msg.velocity[1] = kNaN;
  msg.velocity[2] = kNaN;

  msg.acceleration[0] = kNaN;
  msg.acceleration[1] = kNaN;
  msg.acceleration[2] = kNaN;

  msg.jerk[0] = kNaN;
  msg.jerk[1] = kNaN;
  msg.jerk[2] = kNaN;

  msg.yawspeed = kNaN;

  trajectory_setpoint_pub_->publish(msg);
}

void ArgusCommandController::publishVehicleCommand(uint16_t command, float param1, float param2)
{
  px4_msgs::msg::VehicleCommand msg{};

  msg.timestamp = nowMicros();

  msg.command = command;

  msg.param1 = param1;
  msg.param2 = param2;

  msg.target_system = 1;
  msg.target_component = 1;

  msg.source_system = 1;
  msg.source_component = 1;

  msg.from_external = true;

  vehicle_command_pub_->publish(msg);
}

uint64_t ArgusCommandController::nowMicros()
{
  return static_cast<uint64_t>(now().nanoseconds() / 1000);
}

}  // namespace argus_controller
