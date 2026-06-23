#include <chrono>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include "can_fault_monitor/can_socket.hpp"
#include "can_fault_monitor/msg/can_frame.hpp"

#include <linux/can.h>

using namespace std::chrono_literals;
using CanFrameMsg = can_fault_monitor::msg::CanFrame;

class CanSubscriberNode : public rclcpp::Node
{
public:
  explicit CanSubscriberNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("can_subscriber", options)
  {
    this->declare_parameter<std::string>("interface", "vcan0");
    this->declare_parameter<double>     ("poll_rate", 200.0);

    const std::string iface = this->get_parameter("interface").as_string();
    const double poll_hz    = this->get_parameter("poll_rate").as_double();

    socket_ = std::make_unique<can_fault_monitor::CanSocket>(iface);

    // BEST_EFFORT: high-frequency sensor data — prefer low latency over guaranteed delivery
    frame_pub_ = this->create_publisher<CanFrameMsg>("/can_bus/frames", rclcpp::QoS(100).best_effort());

    const auto period = std::chrono::duration<double>(1.0 / poll_hz);
    poll_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CanSubscriberNode::poll_callback, this));

    RCLCPP_INFO(this->get_logger(),
      "can_subscriber: interface=%s poll=%.0f Hz", iface.c_str(), poll_hz);
  }

private:
  void poll_callback()
  {
    struct can_frame raw{};
    while (socket_->read_frame(raw)) {
      publish_frame(raw);
    }
  }

  void publish_frame(const struct can_frame & raw)
  {
    CanFrameMsg msg;
    msg.stamp           = this->now();
    msg.can_id          = raw.can_id & CAN_EFF_MASK;
    msg.dlc             = raw.can_dlc;
    msg.is_extended_id  = (raw.can_id & CAN_EFF_FLAG) != 0;
    msg.is_remote_frame = (raw.can_id & CAN_RTR_FLAG) != 0;
    msg.is_error_frame  = (raw.can_id & CAN_ERR_FLAG) != 0;

    const uint8_t clamped = std::min(raw.can_dlc, static_cast<uint8_t>(8));
    for (size_t i = 0; i < 8; ++i) msg.data[i] = (i < clamped) ? raw.data[i] : 0;

    // FRAME_CORRUPTION: detect by magic payload signature 0xDE AD BE EF
    // (vcan0 enforces DLC<=8 so we can't use invalid DLC on virtual CAN;
    //  on physical CAN hardware error frames arrive via CAN_ERR_FLAG instead)
    if (!msg.is_error_frame && clamped >= 4 &&
        raw.data[0] == 0xDE && raw.data[1] == 0xAD &&
        raw.data[2] == 0xBE && raw.data[3] == 0xEF) {
      msg.is_error_frame = true;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "[CAN 0x%03X] Corruption signature detected — FRAME_CORRUPTION", msg.can_id);
    }

    // Sequence counter: detect dropped frames via gaps in data[7]
    if (!msg.is_error_frame && clamped == 8) {
      auto it = last_seq_.find(msg.can_id);
      if (it != last_seq_.end()) {
        const uint8_t expected = static_cast<uint8_t>(it->second + 1);
        if (raw.data[7] != expected) {
          RCLCPP_WARN(this->get_logger(),
            "[CAN 0x%03X] Seq gap: expected 0x%02X got 0x%02X",
            msg.can_id, expected, raw.data[7]);
        }
      }
      last_seq_[msg.can_id] = raw.data[7];
    }

    frame_pub_->publish(msg);
  }

  std::unique_ptr<can_fault_monitor::CanSocket> socket_;
  rclcpp::Publisher<CanFrameMsg>::SharedPtr      frame_pub_;
  rclcpp::TimerBase::SharedPtr                   poll_timer_;
  std::unordered_map<uint32_t, uint8_t>          last_seq_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanSubscriberNode>());
  rclcpp::shutdown();
  return 0;
}
