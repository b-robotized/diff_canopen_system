// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>
#include "diff_canopen_system/DiffCanopenSystem.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_canopen_system
{
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn DiffCanopenSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  auto init_rval = CanopenSystem::on_init(info);

  return init_rval;
}

std::vector<hardware_interface::StateInterface> DiffCanopenSystem::export_state_interfaces()
{
  // underlying base class export first
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces = CanopenSystem::export_state_interfaces();
  
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));
    //      RCLCPP_INFO(kLogger, "node id on export state interface for joint: '%s' is '%s'",
    //      info_.joints[i].name.c_str(), info_.joints[i].parameters["node_id"].c_str());


    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "velocity", &wheel_states_[node_id].velcocity_state));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_tempreture", &wheel_states_[node_id].motor_temperature));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_power", &wheel_states_[node_id].motor_power));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "motor_battery_state", &wheel_states_[node_id].motor_battery_state));
    
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "error_status", &wheel_states_[node_id].error_status));
  }

  // Send request

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffCanopenSystem::export_command_interfaces()
{
  auto command_interfaces = CanopenSystem::export_command_interfaces();

  for (uint i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("node_id") == info_.joints[i].parameters.end())
    {
      // skip adding canopen interfaces
      continue;
    }

    const uint8_t node_id = static_cast<uint8_t>(std::stoi(info_.joints[i].parameters["node_id"]));
    
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &wheel_states_[node_id].velocity_command));
  }
  return command_interfaces;
}


hardware_interface::return_type DiffCanopenSystem::read()
{
  auto ret_val = read_velocty_state();
  
  // TODO: Send data to get states...
  send_motor_battery_request();
  auto ret_val_1 = read_motor_battery_states();

  send_motor_status_request();
  auto ret_val_2 = read_error_status();

  send_error_status_request();
  auto ret_val_3 = read_motor_status();

  return ret_val;
}

hardware_interface::return_type DiffCanopenSystem::write()
{
  // Controller use this to connect the hardware...
  // TODO(anyone): write robot's commands'
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

    // reset node nmt
    if (it->second.nmt_state.reset_command())
    {
      it->second.nmt_state.reset_fbk = static_cast<double>(proxy_driver->reset_node_nmt_command());
    }

    // start nmt
    if (it->second.nmt_state.start_command())
    {
      it->second.nmt_state.start_fbk = static_cast<double>(proxy_driver->start_node_nmt_command());
    }

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // TODO(): Convert percents command to speed data
      it->second.tpdo_data.data = convert_percentage_to_speed_value(it->second.tpdo_data.data);
      it->second.tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      // Debug Message
      std::cout << "This is a debug message in HW-write()....." << std::endl;
      std::cout << "Iterator: " << it->first << std::endl;
      std::cout << "Index:    " << it->second.tpdo_data.original_data.index_ << std::endl;
      std::cout << "Subindex: " << it->second.tpdo_data.original_data.subindex_ << std::endl;
      std::cout << "Data:     " << it->second.tpdo_data.original_data.data_ << std::endl;
      std::cout << "Type:     " << it->second.tpdo_data.original_data.type_ << std::endl;
      std::cout << "--- END of the debug message in HW-write()" << std::endl;
    }
  }

  return hardware_interface::return_type::OK;
}

uint32_t DiffCanopenSystem::convert_percentage_to_speed_value(const double percentage) {
  double speed_value_raw = round(percentage * 32767);
  uint32_t speed_value = static_cast<uint32_t>(speed_value_raw);
  return speed_value;
}

double DiffCanopenSystem::convert_rpm_to_rads(const uint32_t rpm) {
  double rpm_raw = static_cast<double>(rpm);
  double rads = rpm_raw/30*M_PI; // = RPM/60*PI*2
  return rads;
}

void DiffCanopenSystem::send_motor_battery_request() {
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // TODO(): Convert percents command to speed data
      it->second.tpdo_data.original_data.index_ = 0x2A6;
      it->second.tpdo_data.original_data.subindex_ = 0;
      // it->second.tpdo_data.original_data.type_ = 0;
      it->second.tpdo_data.original_data.data_ = 0;
      
      // it->second.tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      // Debug Message
      std::cout << "This is a debug message in HW-write()....." << std::endl;
      std::cout << "Iterator: " << it->first << std::endl;
      std::cout << "Index:    " << it->second.tpdo_data.original_data.index_ << std::endl;
      std::cout << "Subindex: " << it->second.tpdo_data.original_data.subindex_ << std::endl;
      std::cout << "Data:     " << it->second.tpdo_data.original_data.data_ << std::endl;
      std::cout << "Type:     " << it->second.tpdo_data.original_data.type_ << std::endl;
      std::cout << "--- END of the debug message in HW-write()" << std::endl;
    }
  }
}

void DiffCanopenSystem::send_error_status_request() {
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // TODO(): Convert percents command to speed data
      it->second.tpdo_data.original_data.index_ = 0x1A6;
      it->second.tpdo_data.original_data.subindex_ = 0;
      // it->second.tpdo_data.original_data.type_ = 0;
      it->second.tpdo_data.original_data.data_ = 0;
      
      // it->second.tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      // Debug Message
      std::cout << "This is a debug message in HW-write()....." << std::endl;
      std::cout << "Iterator: " << it->first << std::endl;
      std::cout << "Index:    " << it->second.tpdo_data.original_data.index_ << std::endl;
      std::cout << "Subindex: " << it->second.tpdo_data.original_data.subindex_ << std::endl;
      std::cout << "Data:     " << it->second.tpdo_data.original_data.data_ << std::endl;
      std::cout << "Type:     " << it->second.tpdo_data.original_data.type_ << std::endl;
      std::cout << "--- END of the debug message in HW-write()" << std::endl;
    }
  }
}

void DiffCanopenSystem::send_motor_status_request() {
  // Motor status via (T)PDO eg 3A6# XX XX YY  YY ZZ ZZ ZZ 00 
  // ( XX XX= Motor RPM; YY YY = temperatur, ZZ ZZ ZZ= Motor Power
  auto drivers = device_container_->get_registered_drivers();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it)
  {
    auto proxy_driver = std::static_pointer_cast<ros2_canopen::ProxyDriver>(drivers[it->first]);

    // tpdo data one shot mechanism
    if (it->second.tpdo_data.write_command())
    {
      // TODO(): Convert percents command to speed data
      it->second.tpdo_data.original_data.index_ = 0x3A6;
      it->second.tpdo_data.original_data.subindex_ = 0;
      // it->second.tpdo_data.original_data.type_ = 0;
      it->second.tpdo_data.original_data.data_ = 0;
      
      // it->second.tpdo_data.prepare_data();
      proxy_driver->tpdo_transmit(it->second.tpdo_data.original_data);
      // Debug Message
      std::cout << "This is a debug message in HW-write()....." << std::endl;
      std::cout << "Iterator: " << it->first << std::endl;
      std::cout << "Index:    " << it->second.tpdo_data.original_data.index_ << std::endl;
      std::cout << "Subindex: " << it->second.tpdo_data.original_data.subindex_ << std::endl;
      std::cout << "Data:     " << it->second.tpdo_data.original_data.data_ << std::endl;
      std::cout << "Type:     " << it->second.tpdo_data.original_data.type_ << std::endl;
      std::cout << "--- END of the debug message in HW-write()" << std::endl;
    }
  }
}

hardware_interface::return_type DiffCanopenSystem::read_motor_battery_states()
{
  send_motor_battery_request();
  auto ret_val = CanopenSystem::read();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
    // TODO(Xi): READ rpdo data
    // hw_states_[it->first] = it->second->rpdo_data.data; 
    std::cout << "This is a debug message in read_motor_battery_states()....." << std::endl;
    std::cout << "Iterator: " << it->first << std::endl;
    std::cout << "Index:    " << it->second.rpdo_data.index << std::endl;
    std::cout << "Subindex: " << it->second.rpdo_data.subindex << std::endl;
    std::cout << "Data:     " << it->second.rpdo_data.data << std::endl;
    std::cout << "Type:     " << it->second.rpdo_data.type << std::endl;
    std::cout << "--- END of the debug message in read_motor_battery_states()" << std::endl;

    // TODO: Get data
    wheel_states_[it->first].motor_battery_state = 0;
  }
  return ret_val;
}

hardware_interface::return_type DiffCanopenSystem::read_error_status()
{
  send_error_status_request();
  auto ret_val = CanopenSystem::read();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
    // TODO(Xi): READ rpdo data
    // hw_states_[it->first] = it->second->rpdo_data.data; 
    std::cout << "This is a debug message in read_error_status_request()....." << std::endl;
    std::cout << "Iterator: " << it->first << std::endl;
    std::cout << "Index:    " << it->second.rpdo_data.index << std::endl;
    std::cout << "Subindex: " << it->second.rpdo_data.subindex << std::endl;
    std::cout << "Data:     " << it->second.rpdo_data.data << std::endl;
    std::cout << "Type:     " << it->second.rpdo_data.type << std::endl;
    std::cout << "--- END of the debug message in read_error_status_request()" << std::endl;

    // TODO(): Get data
    wheel_states_[it->first].error_status = 0;
  }
  return ret_val;
}

hardware_interface::return_type DiffCanopenSystem::read_motor_status()
{
  send_motor_status_request();
  auto ret_val = CanopenSystem::read();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
    // TODO(Xi): READ rpdo data
    // hw_states_[it->first] = it->second->rpdo_data.data; 
    std::cout << "This is a debug message in read_motor_status_request()....." << std::endl;
    std::cout << "Iterator: " << it->first << std::endl;
    std::cout << "Index:    " << it->second.rpdo_data.index << std::endl;
    std::cout << "Subindex: " << it->second.rpdo_data.subindex << std::endl;
    std::cout << "Data:     " << it->second.rpdo_data.data << std::endl;
    std::cout << "Type:     " << it->second.rpdo_data.type << std::endl;
    std::cout << "--- END of the debug message in read_motor_status_request()" << std::endl;

    // TODO(): Get data
    wheel_states_[it->first].velcocity_state = convert_rpm_to_rads(0);
    wheel_states_[it->first].motor_temperature = 0;
    wheel_states_[it->first].motor_power = 0;
  }
  return ret_val;
}

hardware_interface::return_type DiffCanopenSystem::read_velocty_state()
{
  auto ret_val = CanopenSystem::read();
  for (auto it = canopen_data_.begin(); it != canopen_data_.end(); ++it) {
    // TODO(Xi): READ rpdo data
    // hw_states_[it->first] = it->second->rpdo_data.data; 
    std::cout << "This is a debug message in read_velocty_state()....." << std::endl;
    std::cout << "Iterator: " << it->first << std::endl;
    std::cout << "Index:    " << it->second.rpdo_data.index << std::endl;
    std::cout << "Subindex: " << it->second.rpdo_data.subindex << std::endl;
    std::cout << "Data:     " << it->second.rpdo_data.data << std::endl;
    std::cout << "Type:     " << it->second.rpdo_data.type << std::endl;
    std::cout << "--- END of the debug message in read_velocty_state()" << std::endl;

    // TODO(): Get data
    wheel_states_[it->first].velcocity_state = convert_rpm_to_rads(0);
  }
  return ret_val;
}


}  // namespace diff_canopen_system

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  diff_canopen_system::DiffCanopenSystem, hardware_interface::SystemInterface)
