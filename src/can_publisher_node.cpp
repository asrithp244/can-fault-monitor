#include <chrono>
#include <string>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "can_fault_monitor/can_socket.hpp"
#include <linux/can.h>

using namespace std::chrono_literals;

class CanPublisherNode : public rclcpp::Node
{
public:
  explicit CanPublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("can_publisher", options)
  {
    this->declare_parameter<std::string>("interface",    "vcan0");
    this->declare_parameter<int>        ("can_id",       0x100);
    this->declare_parameter<double>     ("publish_rate", 50.0);

    const std::string iface = this->get_parameter("interface").as_string();
    can_id_                 = static_cast<uint32_t>(this->get_parameter("can_id").as_int());
    const double rate_hz    = this->get_parameter("publish_rate").as_double();

    socket_ = std::make_unique<can_fault_monitor::CanSocket>(iface);

    // NODE_DROPOUT: fault injector publishes true here to silence this node
    dropout_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fault_injector/dropout_command", rclcpp::QoS(1).reliable(),
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        in_dropout_.store(msg->data, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(),
          "[CAN 0x%03X] NODE_DROPOUT %s", can_id_, msg->data ? "ARMED" : "CLEARED");
      });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    tx_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanPublisherNode::tx_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "can_publisher: interface=%s CAN_ID=0x%03X rate=%.1f Hz",
      iface.c_str(), can_id_, rate_hz);
  }

private:
  void tx_callback()
  {
    // Under NODE_DROPOUT: stop transmitting — silence is the fault signature
    if (in_dropout_.load(std::memory_order_acquire)) return;

    struct can_frame frame{};
    frame.can_id  = can_id_;
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>((can_id_ >> 8) & 0xFF);
    frame.data[1] = static_cast<uint8_t>( can_id_       & 0xFF);
    frame.data[2] = 0xCA;
    frame.data[3] = 0xFE;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = seq_++;  // rolling counter — subscriber detects drops via gaps

    socket_->write_frame(frame);
  }

  std::unique_ptr<can_fault_monitor::CanSocket>        socket_;
  uint32_t                                             can_id_{0x100};
  uint8_t                                              seq_{0};
  std::atomic<bool>                                    in_dropout_{false};
  rclcpp::TimerBase::SharedPtr                         tx_timer_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dropout_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
