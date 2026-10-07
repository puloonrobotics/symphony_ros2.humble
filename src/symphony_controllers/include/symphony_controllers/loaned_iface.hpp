#ifndef SYMPHONY_CONTROLLERS__LOANED_IFACE_HPP_
#define SYMPHONY_CONTROLLERS__LOANED_IFACE_HPP_

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace symphony_controllers
{

inline void set_command(
  hardware_interface::LoanedCommandInterface & iface, double value)
{
  iface.set_value(value);
}

inline double get_double(
  const hardware_interface::LoanedStateInterface & iface)
{
  return iface.get_value();
}

inline double get_double(
  const hardware_interface::LoanedCommandInterface & iface)
{
  return iface.get_value();
}

}  // namespace symphony_controllers

#endif  // SYMPHONY_CONTROLLERS__LOANED_IFACE_HPP_
