#ifndef ARGUS_CONTROLLER__UDP_COMMAND_RECEIVER_HPP_
#define ARGUS_CONTROLLER__UDP_COMMAND_RECEIVER_HPP_

#include <atomic>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

namespace argus_controller
{

/**
 * @brief UDP -> ROS 2 bridge.
 *
 * Listens on a UDP port (default 5005), receives command strings from the
 * external gesture/voice Python program, and republishes them verbatim on
 * /argus/command for the ARGUS command controller to interpret.
 */
class UdpCommandReceiver : public rclcpp::Node
{
public:
  explicit UdpCommandReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~UdpCommandReceiver() override;

private:
  void receiveLoop();

  int port_;
  double command_cooldown_{1.0};  // seconds between repeats of the same command
  int sock_fd_{-1};
  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
};

}  // namespace argus_controller

#endif  // ARGUS_CONTROLLER__UDP_COMMAND_RECEIVER_HPP_
