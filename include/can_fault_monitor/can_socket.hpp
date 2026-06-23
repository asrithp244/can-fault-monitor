#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>

namespace can_fault_monitor {

/**
 * @brief RAII wrapper around a raw SocketCAN file descriptor.
 *
 * Opens a CAN_RAW socket, resolves the interface index with SIOCGIFINDEX, and
 * binds it — all via direct POSIX syscalls with no libsocketcan dependency.
 * The socket is set non-blocking so ROS 2 timer callbacks can poll without stalling
 * the executor thread.
 *
 * Design decisions:
 *  - PF_CAN / SOCK_RAW / CAN_RAW chosen over SOCK_DGRAM (BCM) because we need
 *    direct access to raw can_frame structs, including the DLC field, for our
 *    FRAME_CORRUPTION detection. BCM filters on content; we need to see malformed frames.
 *  - O_NONBLOCK set at bind time so read() returns EAGAIN instead of blocking.
 *  - No CAN filters installed by default (receives all IDs).
 *  - CAN_RAW_RECV_OWN_MSGS disabled by default. The fault injector enables this
 *    to verify injected frames reach the socket layer.
 */
class CanSocket {
public:
  explicit CanSocket(const std::string & interface);
  ~CanSocket();

  CanSocket(const CanSocket &) = delete;
  CanSocket & operator=(const CanSocket &) = delete;
  CanSocket(CanSocket && other) noexcept;
  CanSocket & operator=(CanSocket && other) noexcept;

  bool write_frame(const struct can_frame & frame);
  bool read_frame(struct can_frame & frame);
  void set_rx_filter(const std::vector<struct can_filter> & filters);
  void set_recv_own_msgs(bool enable);

  int fd() const { return fd_; }

private:
  int fd_{-1};
  void close_fd();
};

}  // namespace can_fault_monitor
