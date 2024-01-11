// Copyright 2023 Stogl Robotics Consulting UG (haftungsbeschränkt)
// All rights reserved.
//
// Made for Robert Bosch GmbH
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

//
// Author: Dr. Denis (denis.stogl@stoglrobotics.de)

#ifndef RPM_POWERTRAIN_DRIVER__RPM_POWERTRAIN_DIFF_CANOPEN_SYSTEM
#define RPM_POWERTRAIN_DRIVER__RPM_POWERTRAIN_DIFF_CANOPEN_SYSTEM

#include <string>
#include <vector>

#include "canopen_ros2_control/canopen_system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rpm_powertrain_driver/visibility_control.h"

namespace rpm_powertrain_driver
{

enum CommandInterfaces
{
  VELOCITY_REFERENCE,
  NMT_RESET,
  NMT_RESET_FBK,
  NMT_START,
  NMT_START_FBK,
};

enum StateInterfaces
{
  VELOCITY_REF,
  VELOCITY_FEEDBACK,
  MOTOR_TEMPERATURE,
  MOTOR_POWER,
  BATTERY_STATE,
  ERROR_STATUS,
  NMT_STATE,
};

struct WheelState {
  // Read only
  double velocity_reference;
  double velocity_feedback;
  double motor_temperature;
  double motor_power;
  double motor_battery_state;
  double error_status; 

  // Write only
  double velocity_command;
};

class RPMPowertrainDiffCanOpenComponent : public canopen_ros2_control::CanopenSystem
{
public:
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  RPMPowertrainDiffCanOpenComponent();
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  ~RPMPowertrainDiffCanOpenComponent() = default;
  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time&, const rclcpp::Duration&) override;

  CANOPEN_ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write();

protected:
  // void initDeviceContainer() override;

private:
  // States - This is a container to store states in a double form
  std::unordered_map<uint, WheelState> wheel_states_;

  // This make the std::pair hashable
  struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1, T2>& pair) const {
        auto h1 = std::hash<T1>{}(pair.first);
        auto h2 = 0;

        // check if T2 is a pair, if so recursively compute hash
        if constexpr (std::is_same<T2, std::pair<uint16_t, uint8_t>>::value) {
            h2 = pair_hash{}(pair.second);
        } else {
            h2 = std::hash<T2>{}(pair.second);
        }

        return h1 ^ h2;
    }
  };

  // PDO_Interfaces_Mapping
  using PDO_INDICES = std::pair<uint16_t, uint8_t>; // Index, Subindex
  using NODE_PDO_INDICES = std::pair<uint, PDO_INDICES>; // Node_id, PDO_INDICES

  // States - Read only
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> state_ro_;

  // Command
  std::unordered_map<NODE_PDO_INDICES, double, pair_hash> velocity_command_; 

  // State converter
  typedef double (*FunctionType)(double);
  std::unordered_map<NODE_PDO_INDICES, FunctionType, pair_hash> state_converter_;

  static double convert_to_position(double rpdo_data);
  static double convert_to_temperature(double rpdo_data);
  static double convert_to_veloctiy(double rpdo_data);
  static double convert_to_RPM(double rpdo_data);
  static double convert_to_switch_voltage(double rpdo_data);

  double convert_rpm_to_rads(const uint32_t rpm);
  double convert_rads_to_rpm(const double rads);
  double convert_rpm_to_percentage(const double rpm);
  uint32_t convert_percentage_to_speed_value(const double percentage);

  std::vector<PDO_INDICES> state_pdo_indices_;

  bool enable_write_ = false;
};

}  // namespace rpm_powertrain_driver

#endif  // RPM_POWERTRAIN_DRIVER__RPM_POWERTRAIN_DIFF_CANOPEN_SYSTEM
