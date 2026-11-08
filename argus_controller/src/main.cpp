#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "argus_controller/argus_command_controller.hpp"
#include "argus_controller/udp_command_receiver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto udp_receiver = std::make_shared<argus_controller::UdpCommandReceiver>();
  auto command_controller = std::make_shared<argus_controller::ArgusCommandController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(udp_receiver);
  executor.add_node(command_controller);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
