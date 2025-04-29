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

#ifndef BASIC_SLAVE_HPP
#define BASIC_SLAVE_HPP
#include <lely/coapp/slave.hpp>
#include <lely/ev/co_task.hpp>
#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>

#include <thread>

#include "canopen_fake_slaves/base_slave.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace lely;
using namespace std::chrono_literals;
namespace ros2_canopen
{

enum ControllerStates {
  POWER_OFF = 0,
  POWER_ON,
  POWER_OFF_CONTACTOR,
  POWER_ON_CONTACTOR,
  POWER_ON_CONTACTOR_BREAK_RELEASE,
  FAULT,
  SAFE_STOP,
};

namespace
{
auto const kLogger = rclcpp::get_logger("FakeMotorController");
}

/**
 * @brief FakeMotorController class simulates a motor controller
 * by sending periodic messages and responding to SDO/RPDO writes.
 */
class FakeMotorController : public canopen::BasicSlave
{
private:
  static const uint16_t RPDO_drive_enable = 0x2100;
  static const uint16_t RPDO_main_contactor = 0x2101;
  static const uint16_t RPDO_break_release = 0x2102;
  static const uint16_t RPDO_safe_stop = 0x2107;
  static const uint16_t RPDO_reset_fault = 0x2109;

  static const uint16_t RPDO_velocity = 0x2110;
  static const uint16_t RPDO_torque = 0x2111;

  static const uint16_t TPDO_power_stage_active = 0x2114;
  static const uint16_t TPDO_fault = 0x2115;
  static const uint16_t TPDO_main_contactor = 0x2117;
  static const uint16_t TPDO_can_enable = 0x2118;
  static const uint16_t TPDO_safe_stop_active = 0x211A;
  static const uint16_t TPDO_toggle_bit = 0x211B;

  static const uint16_t TPDO_load = 0x211B;
  static const uint16_t TPDO_velocity = 0x211C;
  static const uint16_t TPDO_torque = 0x211E;

  static const uint8_t si = 0x00;

public:
  using BasicSlave::BasicSlave;
  ~FakeMotorController()
  {
    if (message_thread.joinable())
    {
      message_thread.join();
    }
  }

protected:
  std::thread message_thread;
  bool toggle_bit_ = false;
  ControllerStates controller_state_ = ControllerStates::FAULT;
  bool last_reset_fault_ = false;

  bool drive_enable_ = false;
  bool main_contactor_ = false;
  bool break_release_ = false;
  bool safe_stop_ = false;
  bool reset_fault_ = false;
  bool can_enable_ = false;

  /**
   * @brief This function is called when a value is written to the local object dictionary by an SDO
   * or RPDO. Also copies the RPDO value to TPDO.
   * @param idx The index of the PDO.
   * @param subidx The si of the PDO.
   */
  void OnWrite(uint16_t idx, uint8_t /*subidx*/) noexcept override
  {
    switch (idx)
    {
      case RPDO_drive_enable:
        drive_enable_ = static_cast<bool>((*this)[idx][si]);
        break;
      case RPDO_main_contactor:
        main_contactor_ = static_cast<bool>((*this)[idx][si]);
        break;
      case RPDO_break_release:
        break_release_ = static_cast<bool>((*this)[idx][si]);
        break;
      case RPDO_safe_stop:
        safe_stop_ = static_cast<bool>((*this)[idx][si]);
        break;
      case RPDO_reset_fault:
        last_reset_fault_ = reset_fault_;
        reset_fault_ = static_cast<bool>((*this)[idx][si]);
        break;
      default:
        break;
    }

    // Publish periodic message
    if (!message_thread.joinable())
    {
      message_thread = std::thread(std::bind(&FakeMotorController::update_toogle_bit_periodically, this));
    }
  }

  /**
   * @brief This function is attached to a thread and sends periodic messages
   */
  void update_toogle_bit_periodically()
  {
    // If ros is running, send messages
    while (rclcpp::ok())
    {
      // don't communicate until you get the reset_fault command
      if ((controller_state_ == ControllerStates::FAULT || controller_state_ == ControllerStates::SAFE_STOP) && !last_reset_fault_ && reset_fault_)
      {
        RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_OFF.", static_cast<uint8_t>(id()));
        controller_state_ = ControllerStates::POWER_OFF;
      }
      else
      {
        RCLCPP_INFO(kLogger, "Nodfe 0x%X not yet resetted.", static_cast<uint8_t>(id()));
        return;
      }

      // CAN is always enabled
      (*this)[TPDO_can_enable][si] = true;
      (*this)[TPDO_safe_stop_active][si] = safe_stop_;

      if (safe_stop_)
      {
        (*this)[TPDO_fault][si] = true;
        (*this)[TPDO_power_stage_active][si] = false;
        (*this)[TPDO_main_contactor][si] = false;
        (*this)[TPDO_velocity][si] = 0.0;
        (*this)[TPDO_torque][si] = 0.0;
        controller_state_ = ControllerStates::SAFE_STOP;
      }
      else
      {
        if (!drive_enable_)
        {
          if (!main_contactor_)
          {
            controller_state_ = ControllerStates::POWER_OFF;
            (*this)[TPDO_power_stage_active][si] = false;
            (*this)[TPDO_main_contactor][si] = false;
          }
          else
          {
            controller_state_ = ControllerStates::POWER_OFF_CONTACTOR;
            (*this)[TPDO_power_stage_active][si] = false;
            (*this)[TPDO_main_contactor][si] = true;
          }
        }

        switch(controller_state_)
        {
          case ControllerStates::POWER_OFF:
            if (drive_enable_)
            {
              RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_ON.", static_cast<uint8_t>(id()));
              controller_state_ = ControllerStates::POWER_ON;
              (*this)[TPDO_power_stage_active][si] = true;
              (*this)[TPDO_main_contactor][si] = false;
            }
            break;
          case ControllerStates::POWER_OFF_CONTACTOR:
            if (drive_enable_ && main_contactor_)
            {
              RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_ON_CONTACTOR.", static_cast<uint8_t>(id()));
              controller_state_ = ControllerStates::POWER_ON_CONTACTOR;
              (*this)[TPDO_power_stage_active][si] = true;
              (*this)[TPDO_main_contactor][si] = true;
          }
            break;
          case ControllerStates::POWER_ON:
            if (drive_enable_ && main_contactor_)
            {
              RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_ON_CONTACTOR.", static_cast<uint8_t>(id()));
              controller_state_ = ControllerStates::POWER_ON_CONTACTOR;
              (*this)[TPDO_power_stage_active][si] = true;
              (*this)[TPDO_main_contactor][si] = true;
            }
            break;
          case ControllerStates::POWER_ON_CONTACTOR:
            if (drive_enable_ && main_contactor_ && break_release_)
            {
              RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_ON_CONTACTOR_BREAK_RELEASE.", static_cast<uint8_t>(id()));
              controller_state_ = ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE;
              (*this)[TPDO_power_stage_active][si] = true;
              (*this)[TPDO_main_contactor][si] = true;
            }
            break;
          case ControllerStates::POWER_ON_CONTACTOR_BREAK_RELEASE:
            if (drive_enable_ && main_contactor_ && !break_release_)
            {
              RCLCPP_INFO(kLogger, "Nodfe 0x%X switching to state POWER_ON_CONTACTOR.", static_cast<uint8_t>(id()));
              controller_state_ = ControllerStates::POWER_ON_CONTACTOR;
              (*this)[TPDO_power_stage_active][si] = true;
              (*this)[TPDO_main_contactor][si] = true;
            }
            else
            {  // check that the break is released when velocity is set
              (*this)[TPDO_velocity ][si] = (*this)[RPDO_velocity][si];
              (*this)[TPDO_torque][si] = (*this)[RPDO_torque][si];
            }
            break;
          default:
            break;
          };
      }

      (*this)[TPDO_toggle_bit][si] = toggle_bit_;
      toggle_bit_ = !toggle_bit_;
      this->TpdoEvent(0);
      // 20 ms sleep - 50 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
};

class BasicSlave : public BaseSlave
{
public:
  explicit BasicSlave(const std::string & node_name, bool intra_process_comms = false)
  : BaseSlave(node_name, intra_process_comms)
  {
  }

protected:
  class ActiveCheckTask : public ev::CoTask
  {
  public:
    ActiveCheckTask(io::Context * ctx, ev::Executor * execl, BasicSlave * slave) : CoTask(*execl)
    {
      slave_ = slave;
      exec_ = execl;
      ctx_ = ctx;
    }

  protected:
    BasicSlave * slave_;
    ev::Executor * exec_;
    io::Context * ctx_;
    virtual void operator()() noexcept
    {
      if (slave_->activated.load())
      {
      }
      ctx_->shutdown();
    }
  };

  void run() override
  {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl(can_interface_name_.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    auto sigset_ = lely::io::SignalSet(poll, exec);
    // Watch for Ctrl+C or process termination.
    sigset_.insert(SIGHUP);
    sigset_.insert(SIGINT);
    sigset_.insert(SIGTERM);

    sigset_.submit_wait(
      [&](int /*signo*/)
      {
        // If the signal is raised again, terminate immediately.
        sigset_.clear();

        // Perform a clean shutdown.
        ctx.shutdown();
      });

    // TODO(drdenis): Make this fix in the BaseSlave
    FakeMotorController slave(timer, chan, slave_config_.c_str(), "", static_cast<uint8_t>(node_id_));
    slave.Reset();
    ActiveCheckTask checktask(&ctx, &exec, this);

    // timer.submit_wait()
    RCLCPP_INFO(this->get_logger(), "Created slave for node_id 0x%X.", static_cast<uint8_t>(node_id_));
    loop.run();
    ctx.shutdown();
    RCLCPP_INFO(this->get_logger(), "Stopped CANopen Event Loop.");
    rclcpp::shutdown();
  }
};
}  // namespace ros2_canopen

#endif
