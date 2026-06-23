#include "can_fault_monitor/can_socket.hpp"

#include <cstring>
#include <cerrno>
#include <stdexcept>
#include <system_error>

#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace can_fault_monitor {

CanSocket::CanSocket(const std::string & interface)
{
  // Step 1: Open a raw CAN socket.
  // PF_CAN / SOCK_RAW / CAN_RAW — raw frame access, no libsocketcan dependency.
  fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd_ < 0) {
    throw std::system_error(errno, std::generic_category(),
      "socket(PF_CAN, SOCK_RAW, CAN_RAW) failed. "
      "Run: sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0");
  }

  // Step 2: Set non-blocking I/O so read() returns EAGAIN if no frame is available.
  // Required because we run inside a ROS 2 wall-timer callback.
  int flags = ::fcntl(fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close_fd();
    throw std::system_error(errno, std::generic_category(), "fcntl(O_NONBLOCK) failed");
  }

  // Step 3: Resolve interface name → index via SIOCGIFINDEX ioctl.
  struct ifreq ifr;
  std::memset(&ifr, 0, sizeof(ifr));
  std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd_, SIOCGIFINDEX, &ifr) < 0) {
    close_fd();
    throw std::system_error(errno, std::generic_category(),
      "ioctl(SIOCGIFINDEX) failed for interface '" + interface + "'");
  }

  // Step 4: Bind socket to the interface.
  struct sockaddr_can addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    close_fd();
    throw std::system_error(errno, std::generic_category(),
      "bind() failed for interface '" + interface + "'");
  }
}

CanSocket::~CanSocket() { close_fd(); }

CanSocket::CanSocket(CanSocket && other) noexcept : fd_(other.fd_) { other.fd_ = -1; }

CanSocket & CanSocket::operator=(CanSocket && other) noexcept
{
  if (this != &other) { close_fd(); fd_ = other.fd_; other.fd_ = -1; }
  return *this;
}

bool CanSocket::write_frame(const struct can_frame & frame)
{
  ssize_t nbytes = ::write(fd_, &frame, sizeof(struct can_frame));
  if (nbytes == static_cast<ssize_t>(sizeof(struct can_frame))) return true;
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == ENETDOWN) return false;
    throw std::system_error(errno, std::generic_category(), "CAN write() failed");
  }
  throw std::runtime_error("CAN write(): partial write");
}

bool CanSocket::read_frame(struct can_frame & frame)
{
  ssize_t nbytes = ::read(fd_, &frame, sizeof(struct can_frame));
  if (nbytes == static_cast<ssize_t>(sizeof(struct can_frame))) return true;
  if (nbytes < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return false;
    throw std::system_error(errno, std::generic_category(), "CAN read() failed");
  }
  return false;
}

void CanSocket::set_rx_filter(const std::vector<struct can_filter> & filters)
{
  if (filters.empty()) {
    ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER, nullptr, 0);
    return;
  }
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
               filters.data(),
               static_cast<socklen_t>(filters.size() * sizeof(struct can_filter)));
}

void CanSocket::set_recv_own_msgs(bool enable)
{
  int opt = enable ? 1 : 0;
  ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &opt, sizeof(opt));
}

void CanSocket::close_fd()
{
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

}  // namespace can_fault_monitor
