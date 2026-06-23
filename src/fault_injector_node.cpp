#include <chrono>
#include <string>
#include <atomic>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "can_fault_monitor/can_socket.hpp"
#include "can_fault_monitor/fault_types.hpp"
#include "can_fault_monitor_msgs/msg/fault_event.hpp"
#include "can_fault_monitor_msgs/srv/inject_fault.hpp"

#include <linux/can.h>

using namespace std::chrono_literals;
using InjectFault = can_fault_monitor_msgs::srv::InjectFault;
using FaultEvent  = can_fault_monitor_msgs::msg::FaultEvent;
using can_fault_monitor::FaultType;
using can_fault_monitor::fault_type_from_string;
using can_fault_monitor::fault_type_to_string;

class FaultInjectorNode : public rclcpp::Node
{
public:
  explicit FaultInjectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fault_injector", options)
  {
    this->declare_parameter<std::string>("interface",            "vcan0");
    this->declare_parameter<int>        ("flood_can_id",         0x7FF);
    this->declare_parameter<double>     ("flood_rate_hz",        2000.0);
    this->declare_parameter<double>     ("corruption_rate_hz",   20.0);
    this->declare_parameter<int>        ("corruption_target_id", 0x100);

    const std::string iface  = this->get_parameter("interface").as_string();
    flood_can_id_            = static_cast<uint32_t>(this->get_parameter("flood_can_id").as_int());
    flood_rate_hz_           = this->get_parameter("flood_rate_hz").as_double();
    corruption_rate_hz_      = this->get_parameter("corruption_rate_hz").as_double();
    corruption_target_id_    = static_cast<uint32_t>(this->get_parameter("corruption_target_id").as_int());

    socket_ = std::make_unique<can_fault_monitor::CanSocket>(iface);
    socket_->set_recv_own_msgs(true);  // verify injected frames reach the kernel layer

    dropout_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/fault_injector/dropout_command", rclcpp::QoS(1).reliable());
    event_pub_ = this->create_publisher<FaultEvent>(
      "/fault_injector/events", rclcpp::QoS(100).reliable());

    service_ = this->create_service<InjectFault>(
      "/fault_injector/inject_fault",
      std::bind(&FaultInjectorNode::handle_inject, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "fault_injector ready.");
  }

private:
  void handle_inject(
    const std::shared_ptr<InjectFault::Request>  req,
    const std::shared_ptr<InjectFault::Response> res)
  {
    FaultType ft;
    try { ft = fault_type_from_string(req->fault_type); }
    catch (const std::invalid_argument & e) { res->success = false; res->message = e.what(); return; }

    cancel_active_fault();
    active_fault_ = ft;

    RCLCPP_WARN(this->get_logger(), "Injecting: %s (%.2f sec)", fault_type_to_string(ft), req->duration_sec);
    publish_event("INJECTED", fault_type_to_string(ft), req->target_can_id, 0.0);
    res->injection_timestamp = this->now().seconds();

    switch (ft) {
      case FaultType::NODE_DROPOUT:     arm_node_dropout(req->target_can_id, req->duration_sec); break;
      case FaultType::FRAME_CORRUPTION: arm_frame_corruption(req->target_can_id, req->duration_sec); break;
      case FaultType::BUS_FLOOD:        arm_bus_flood(req->duration_sec); break;
      default: break;
    }

    res->success = true;
    res->message = std::string("Armed: ") + fault_type_to_string(ft);
  }

  void arm_node_dropout(uint32_t target_id, double duration)
  {
    dropout_target_id_ = target_id;
    auto msg = std::make_unique<std_msgs::msg::Bool>(); msg->data = true;
    dropout_pub_->publish(std::move(msg));
    if (duration > 0.0) {
      stop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(duration)),
        [this]{ stop_timer_->cancel(); cancel_node_dropout(); });
    }
  }

  void cancel_node_dropout()
  {
    auto msg = std::make_unique<std_msgs::msg::Bool>(); msg->data = false;
    dropout_pub_->publish(std::move(msg));
  }

  void arm_frame_corruption(uint32_t target_id, double duration)
  {
    corruption_target_id_ = target_id;
    const auto p = std::chrono::duration<double>(1.0 / corruption_rate_hz_);
    inject_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(p),
      [this]{ send_corrupt_frame(); });
    if (duration > 0.0) {
      stop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(duration)),
        [this]{ stop_timer_->cancel(); if (inject_timer_) { inject_timer_->cancel(); inject_timer_.reset(); } active_fault_ = FaultType::NONE; });
    }
  }

  void send_corrupt_frame()
  {
    struct can_frame frame{};
    frame.can_id  = corruption_target_id_;
    frame.can_dlc = 9;  // ISO 11898 violation: max DLC is 8
    frame.data[0] = 0xDE; frame.data[1] = 0xAD;
    frame.data[2] = 0xBE; frame.data[3] = 0xEF;
    std::fill(frame.data + 4, frame.data + 8, 0xFF);
    socket_->write_frame(frame);
  }

  void arm_bus_flood(double duration)
  {
    const auto p = std::chrono::duration<double>(1.0 / flood_rate_hz_);
    inject_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(p),
      [this]{ send_flood_frame(); });
    if (duration > 0.0) {
      stop_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(duration)),
        [this]{ stop_timer_->cancel(); if (inject_timer_) { inject_timer_->cancel(); inject_timer_.reset(); } active_fault_ = FaultType::NONE; });
    }
  }

  void send_flood_frame()
  {
    struct can_frame frame{};
    frame.can_id  = flood_can_id_;  // 0x7FF = lowest CAN priority
    frame.can_dlc = 8;
    std::fill(std::begin(frame.data), std::end(frame.data), 0xFF);
    socket_->write_frame(frame);
  }

  void cancel_active_fault()
  {
    if (inject_timer_) { inject_timer_->cancel(); inject_timer_.reset(); }
    if (stop_timer_)   { stop_timer_->cancel();   stop_timer_.reset();   }
    if (active_fault_ == FaultType::NODE_DROPOUT) cancel_node_dropout();
    active_fault_ = FaultType::NONE;
  }

  void publish_event(const std::string & type, const std::string & fault, uint32_t id, double lat)
  {
    FaultEvent ev; ev.stamp = this->now(); ev.event_type = type;
    ev.fault_type = fault; ev.affected_can_id = id; ev.latency_ms = lat;
    event_pub_->publish(ev);
  }

  std::unique_ptr<can_fault_monitor::CanSocket>      socket_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr  dropout_pub_;
  rclcpp::Publisher<FaultEvent>::SharedPtr           event_pub_;
  rclcpp::Service<InjectFault>::SharedPtr            service_;
  rclcpp::TimerBase::SharedPtr inject_timer_, stop_timer_;

  FaultType active_fault_{FaultType::NONE};
  uint32_t  flood_can_id_{0x7FF};
  double    flood_rate_hz_{2000.0};
  double    corruption_rate_hz_{20.0};
  uint32_t  corruption_target_id_{0x100};
  uint32_t  dropout_target_id_{0x100};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultInjectorNode>());
  rclcpp::shutdown();
  return 0;
}
