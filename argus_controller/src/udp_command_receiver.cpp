#include "argus_controller/udp_command_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace argus_controller
{

namespace
{
// ============================================================================
// Command repeat cooldown -- edit this and rebuild to change behaviour.
// ============================================================================
// The gesture/voice program sends the same command continuously while a
// gesture is held (e.g. UP, UP, UP, ...). After a command is forwarded, any
// repeat of the SAME command is dropped until this many seconds have passed,
// so holding a gesture produces one step every kCommandCooldown seconds
// instead of a continuous stream. A DIFFERENT command is always forwarded
// immediately. Set to 0.0 to disable the filter.
constexpr double kCommandCooldown = 1.0;
}  // namespace

UdpCommandReceiver::UdpCommandReceiver(const rclcpp::NodeOptions & options)
: Node("udp_command_receiver", options)
{
  port_ = declare_parameter<int>("udp_port", 5005);
  // ROS parameter of the same name (if supplied) overrides the constant above.
  command_cooldown_ = declare_parameter<double>("command_cooldown", kCommandCooldown);

  command_pub_ = create_publisher<std_msgs::msg::String>("/argus/command", rclcpp::QoS(10));

  sock_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock_fd_ < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to create UDP socket: %s", std::strerror(errno));
    throw std::runtime_error("socket() failed");
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(static_cast<uint16_t>(port_));

  if (::bind(sock_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_FATAL(get_logger(), "Failed to bind UDP port %d: %s", port_, std::strerror(errno));
    ::close(sock_fd_);
    sock_fd_ = -1;
    throw std::runtime_error("bind() failed");
  }

  // 1 s receive timeout so the thread can notice shutdown.
  timeval tv{};
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  ::setsockopt(sock_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  running_ = true;
  rx_thread_ = std::thread(&UdpCommandReceiver::receiveLoop, this);

  RCLCPP_INFO(get_logger(), "[ARGUS] UDP receiver listening on 0.0.0.0:%d -> /argus/command", port_);
  RCLCPP_INFO(get_logger(), "[ARGUS] Repeated-command cooldown: %.2f s", command_cooldown_);
}

UdpCommandReceiver::~UdpCommandReceiver()
{
  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }
}

void UdpCommandReceiver::receiveLoop()
{
  using Clock = std::chrono::steady_clock;

  char buffer[1024];
  std::string last_cmd;
  Clock::time_point last_sent_time{};
  bool have_last = false;

  while (running_ && rclcpp::ok()) {
    const ssize_t n = ::recvfrom(sock_fd_, buffer, sizeof(buffer) - 1, 0, nullptr, nullptr);
    if (n <= 0) {
      continue;  // timeout or interrupted; re-check running_
    }
    buffer[n] = '\0';

    std_msgs::msg::String msg;
    msg.data = std::string(buffer, static_cast<size_t>(n));

    // Normalised key (trimmed, upper-case) used only for repeat detection.
    std::string key = msg.data;
    key.erase(std::remove_if(key.begin(), key.end(),
      [](unsigned char c) { return std::isspace(c); }), key.end());
    std::transform(key.begin(), key.end(), key.begin(),
      [](unsigned char c) { return std::toupper(c); });
    if (key.empty()) {
      continue;
    }

    // Drop repeats of the same command inside the cooldown window.
    const auto now = Clock::now();
    if (command_cooldown_ > 0.0 && have_last && key == last_cmd &&
      std::chrono::duration<double>(now - last_sent_time).count() < command_cooldown_)
    {
      continue;
    }
    last_cmd = key;
    last_sent_time = now;
    have_last = true;

    command_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "[ARGUS] UDP received, forwarding to /argus/command: '%s'", msg.data.c_str());
  }
}

}  // namespace argus_controller
