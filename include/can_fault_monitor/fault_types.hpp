#pragma once

#include <string>
#include <stdexcept>

namespace can_fault_monitor {

enum class FaultType : uint8_t {
  NONE             = 0,
  NODE_DROPOUT     = 1,
  FRAME_CORRUPTION = 2,
  BUS_FLOOD        = 3,
};

inline FaultType fault_type_from_string(const std::string & s)
{
  if (s == "NODE_DROPOUT")      return FaultType::NODE_DROPOUT;
  if (s == "FRAME_CORRUPTION")  return FaultType::FRAME_CORRUPTION;
  if (s == "BUS_FLOOD")         return FaultType::BUS_FLOOD;
  throw std::invalid_argument("Unknown fault type: " + s);
}

inline const char * fault_type_to_string(FaultType ft)
{
  switch (ft) {
    case FaultType::NODE_DROPOUT:      return "NODE_DROPOUT";
    case FaultType::FRAME_CORRUPTION:  return "FRAME_CORRUPTION";
    case FaultType::BUS_FLOOD:         return "BUS_FLOOD";
    default:                           return "NONE";
  }
}

enum class BusState : uint8_t {
  NOMINAL  = 0,
  DEGRADED = 1,
  RECOVERY = 2,
};

inline const char * bus_state_to_string(BusState s)
{
  switch (s) {
    case BusState::NOMINAL:  return "NOMINAL";
    case BusState::DEGRADED: return "DEGRADED";
    case BusState::RECOVERY: return "RECOVERY";
    default:                 return "UNKNOWN";
  }
}

}  // namespace can_fault_monitor
