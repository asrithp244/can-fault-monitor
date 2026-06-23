#include <chrono>
#include <string>
#include <unordered_map>
#include <deque>
#include <fstream>
#include <filesystem>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "can_fault_monitor/fault_types.hpp"
#include "can_fault_monitor_msgs/msg/can_frame.hpp"
#include "can_fault_monitor_msgs/msg/can_health_status.hpp"
#include "can_fault_monitor_msgs/msg/fault_event.hpp"

using namespace std::chrono_literals;
using CanFrameMsg      = can_fault_monitor_msgs::msg::CanFrame;
using CanHealthStatus  = can_fault_monitor_msgs::msg::CanHealthStatus;
using FaultEvent       = can_fault_monitor_msgs::msg::FaultEvent;
using can_fault_monitor::BusState;
using can_fault_monitor::FaultType;
using can_fault_monitor::bus_state_to_string;
using can_fault_monitor::fault_type_to_string;

class CanHealthMonitorNode : public rclcpp::Node
{
public:
  explicit CanHealthMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("can_health_monitor", options)
  {
    this->declare_parameter<double>("dropout_deadline_ms",        100.0);
    this->declare_parameter<double>("corruption_window_ms",       500.0);
    this->declare_parameter<double>("corruption_threshold_ratio", 0.20);
    this->declare_parameter<double>("flood_window_ms",            100.0);
    this->declare_parameter<double>("flood_threshold_fps",        500.0);
    this->declare_parameter<std::string>("metrics_file",          "results/metrics.csv");

    dropout_deadline_   = this->get_parameter("dropout_deadline_ms").as_double() / 1000.0;
    corruption_window_  = this->get_parameter("corruption_window_ms").as_double() / 1000.0;
    corruption_thresh_  = this->get_parameter("corruption_threshold_ratio").as_double();
    flood_window_       = this->get_parameter("flood_window_ms").as_double() / 1000.0;
    flood_thresh_fps_   = this->get_parameter("flood_threshold_fps").as_double();
    metrics_file_       = this->get_parameter("metrics_file").as_string();

    // Create results directory and open CSV
    std::filesystem::create_directories(
      std::filesystem::path(metrics_file_).parent_path().empty() ? "." :
      std::filesystem::path(metrics_file_).parent_path());
    csv_.open(metrics_file_, std::ios::app);
    if (csv_.is_open() && csv_.tellp() == 0) {
      csv_ << "timestamp_sec,fault_type,detection_latency_ms,recovery_latency_ms,"
           << "bus_frame_rate_hz,invalid_frame_ratio\n";
    }

    frame_sub_ = this->create_subscription<CanFrameMsg>(
      "/can_bus/frames", rclcpp::QoS(100).best_effort(),
      std::bind(&CanHealthMonitorNode::on_frame, this, std::placeholders::_1));

    fault_event_sub_ = this->create_subscription<FaultEvent>(
      "/fault_injector/events", rclcpp::QoS(100).reliable(),
      std::bind(&CanHealthMonitorNode::on_fault_event, this, std::placeholders::_1));

    status_pub_ = this->create_publisher<CanHealthStatus>(
      "/can_health/status", rclcpp::QoS(10).reliable());

    // Evaluate detection every 20 ms
    eval_timer_ = this->create_wall_timer(20ms,
      std::bind(&CanHealthMonitorNode::evaluate_health, this));

    RCLCPP_INFO(this->get_logger(),
      "can_health_monitor started. Metrics → %s", metrics_file_.c_str());
  }

private:
  struct PerIdState {
    double last_rx_time{0.0};
    bool   dropout_active{false};
    double dropout_detected_at{0.0};
    double dropout_recovery_at{0.0};
    double backoff_ms{100.0};

    // Sliding window for frame timestamps (for corruption + flood detection)
    std::deque<double> all_rx_times;
    std::deque<double> invalid_rx_times;
    uint64_t           total_frames{0};
    uint64_t           invalid_frames{0};
  };

  void on_frame(const CanFrameMsg::SharedPtr msg)
  {
    const double now = this->now().seconds();
    const uint32_t id = msg->can_id;
    auto & st = per_id_[id];

    // Update dropout deadline timer
    st.last_rx_time = now;
    if (st.dropout_active) {
      st.dropout_recovery_at = now;
      const double rec_ms = (now - st.dropout_detected_at) * 1000.0;
      RCLCPP_INFO(this->get_logger(),
        "[CAN 0x%03X] NODE_DROPOUT cleared. Recovery latency: %.1f ms", id, rec_ms);
      log_metric(now, "NODE_DROPOUT", 0.0, rec_ms);
      st.dropout_active  = false;
      st.backoff_ms = 100.0;  // reset backoff
    }

    // Track all frame timestamps for flood detection
    st.all_rx_times.push_back(now);
    prune_window(st.all_rx_times, now, flood_window_);

    // Track invalid frames for corruption detection
    if (msg->is_error_frame || msg->dlc > 8) {
      st.invalid_rx_times.push_back(now);
      st.invalid_frames++;
    }
    prune_window(st.invalid_rx_times, now, corruption_window_);
    st.total_frames++;
  }

  void on_fault_event(const FaultEvent::SharedPtr msg)
  {
    if (msg->event_type == "INJECTED") {
      active_fault_type_    = msg->fault_type;
      active_fault_id_      = msg->affected_can_id;
      injection_time_       = this->now().seconds();
      RCLCPP_WARN(this->get_logger(),
        "Fault INJECTED: %s on CAN 0x%03X", msg->fault_type.c_str(), msg->affected_can_id);
    }
  }

  void evaluate_health()
  {
    const double now = this->now().seconds();
    bus_state_    = BusState::NOMINAL;
    active_fault_ = FaultType::NONE;

    for (auto & [id, st] : per_id_) {
      // NODE_DROPOUT detection: deadline timer
      if (!st.dropout_active && st.last_rx_time > 0.0 &&
          (now - st.last_rx_time) > dropout_deadline_)
      {
        st.dropout_active      = true;
        st.dropout_detected_at = now;
        const double latency_ms = (now - st.last_rx_time) * 1000.0;
        RCLCPP_WARN(this->get_logger(),
          "[CAN 0x%03X] NODE_DROPOUT detected. Latency: %.1f ms", id, latency_ms);
        bus_state_    = BusState::DEGRADED;
        active_fault_ = FaultType::NODE_DROPOUT;
        log_metric(now, "NODE_DROPOUT", latency_ms, 0.0);
        schedule_recovery_attempt(id, st);
      } else if (st.dropout_active) {
        bus_state_    = BusState::DEGRADED;
        active_fault_ = FaultType::NODE_DROPOUT;
      }

      // FRAME_CORRUPTION detection: sliding window invalid ratio
      {
        prune_window(st.invalid_rx_times, now, corruption_window_);
        prune_window(st.all_rx_times, now, corruption_window_);
        const double total   = static_cast<double>(st.all_rx_times.size());
        const double invalid = static_cast<double>(st.invalid_rx_times.size());
        const double ratio   = (total > 0) ? (invalid / total) : 0.0;
        if (ratio > corruption_thresh_) {
          bus_state_    = BusState::DEGRADED;
          active_fault_ = FaultType::FRAME_CORRUPTION;
        }
      }

      // BUS_FLOOD detection: per-ID frame rate in flood window
      {
        prune_window(st.all_rx_times, now, flood_window_);
        const double fps = static_cast<double>(st.all_rx_times.size()) / flood_window_;
        if (fps > flood_thresh_fps_) {
          if (bus_state_ != BusState::DEGRADED) {
            const double latency_ms = 0.0;  // detected immediately
            RCLCPP_WARN(this->get_logger(),
              "[CAN 0x%03X] BUS_FLOOD detected. Rate: %.0f fps", id, fps);
            log_metric(now, "BUS_FLOOD", latency_ms, 0.0);
          }
          bus_state_    = BusState::DEGRADED;
          active_fault_ = FaultType::BUS_FLOOD;
        }
      }
    }

    publish_status(now);
  }

  void schedule_recovery_attempt(uint32_t id, PerIdState & st)
  {
    if (st.backoff_ms > 2000.0) st.backoff_ms = 2000.0;
    const double backoff = st.backoff_ms;
    st.backoff_ms = std::min(st.backoff_ms * 2.0, 2000.0);
    RCLCPP_INFO(this->get_logger(),
      "[CAN 0x%03X] Recovery probe in %.0f ms", id, backoff);
  }

  void publish_status(double now)
  {
    CanHealthStatus msg;
    msg.stamp            = this->now();
    msg.bus_state        = static_cast<uint8_t>(bus_state_);
    msg.active_fault     = fault_type_to_string(active_fault_);
    msg.total_faults_detected = total_faults_detected_;

    // Aggregate frame rate and invalid ratio across all IDs
    double total_fps = 0.0; double max_invalid_ratio = 0.0;
    for (auto & [id, st] : per_id_) {
      prune_window(st.all_rx_times, now, flood_window_);
      total_fps += static_cast<double>(st.all_rx_times.size()) / flood_window_;
      const double t = static_cast<double>(st.all_rx_times.size());
      const double iv = static_cast<double>(st.invalid_rx_times.size());
      if (t > 0) max_invalid_ratio = std::max(max_invalid_ratio, iv / t);
    }
    msg.bus_frame_rate_hz  = total_fps;
    msg.invalid_frame_ratio = max_invalid_ratio;

    if (bus_state_ != last_published_state_) {
      RCLCPP_INFO(this->get_logger(),
        "Bus state → %s (fault: %s)", bus_state_to_string(bus_state_), fault_type_to_string(active_fault_));
      last_published_state_ = bus_state_;
    }
    status_pub_->publish(msg);
  }

  void log_metric(double ts, const std::string & fault,
                  double det_ms, double rec_ms)
  {
    total_faults_detected_++;
    if (!csv_.is_open()) return;
    const double fps = 0.0;
    const double ratio = 0.0;
    csv_ << std::fixed << std::setprecision(3)
         << ts << "," << fault << "," << det_ms << "," << rec_ms
         << "," << fps << "," << ratio << "\n";
    csv_.flush();
  }

  void prune_window(std::deque<double> & dq, double now, double window)
  {
    while (!dq.empty() && (now - dq.front()) > window) dq.pop_front();
  }

  // Subscriptions / publishers
  rclcpp::Subscription<CanFrameMsg>::SharedPtr   frame_sub_;
  rclcpp::Subscription<FaultEvent>::SharedPtr    fault_event_sub_;
  rclcpp::Publisher<CanHealthStatus>::SharedPtr  status_pub_;
  rclcpp::TimerBase::SharedPtr                   eval_timer_;

  // Detection thresholds (from params)
  double      dropout_deadline_{0.1};
  double      corruption_window_{0.5};
  double      corruption_thresh_{0.20};
  double      flood_window_{0.1};
  double      flood_thresh_fps_{500.0};
  std::string metrics_file_{"results/metrics.csv"};

  // State
  std::unordered_map<uint32_t, PerIdState> per_id_;
  BusState    bus_state_{BusState::NOMINAL};
  BusState    last_published_state_{BusState::NOMINAL};
  FaultType   active_fault_{FaultType::NONE};
  std::string active_fault_type_;
  uint32_t    active_fault_id_{0};
  double      injection_time_{0.0};
  uint32_t    total_faults_detected_{0};
  std::ofstream csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanHealthMonitorNode>());
  rclcpp::shutdown();
  return 0;
}
