//    Copyright 2022 Christoph Hellmann Santos
//
//    Licensed under the Apache License, Version 2.0 (the "License");
//    you may not use this file except in compliance with the License.
//    You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
//    Unless required by applicable law or agreed to in writing, software
//    distributed under the License is distributed on an "AS IS" BASIS,
//    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//    See the License for the specific language governing permissions and
//    limitations under the License.

#include "fake_canopen/fake_motor_controller.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::InitOptions options;
  options.shutdown_on_signal = true;
  // rclcpp::init(argc, argv, options, rclcpp::SignalHandlerOptions::All);
  rclcpp::init(argc, argv, options);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto canopen_slave = std::make_shared<ros2_canopen::BasicSlave>("Fake_Motor_Controller");
  executor.add_node(canopen_slave->get_node_base_interface());
  executor.spin();
  return 0;
}
