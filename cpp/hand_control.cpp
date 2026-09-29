#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <unitree/idl/hg/HandCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>

#include "hand_action_lcmt.hpp"

namespace {

constexpr std::size_t kMotorCount = 7;
constexpr std::chrono::milliseconds kControlPeriod(10);  // 100 Hz

constexpr std::array<float, kMotorCount> kLeftMin = {
    -1.05F, -0.724F, 0.0F, -1.57F, -1.75F, -1.57F, -1.75F};
constexpr std::array<float, kMotorCount> kLeftMax = {
    1.05F, 1.05F, 1.75F, 0.0F, 0.0F, 0.0F, 0.0F};
constexpr std::array<float, kMotorCount> kRightMin = {
    -1.05F, -1.05F, -1.75F, 0.0F, 0.0F, 0.0F, 0.0F};
constexpr std::array<float, kMotorCount> kRightMax = {
    1.05F, 0.742F, 0.0F, 1.57F, 1.75F, 1.57F, 1.75F};

constexpr const char* kLeftCommandTopic = "rt/dex3/left/cmd";
constexpr const char* kRightCommandTopic = "rt/dex3/right/cmd";
constexpr const char* kDefaultHandActionChannel = "hand_action";

uint8_t MakeMotorMode(std::size_t motor_id) {
  constexpr uint8_t kEnabledStatus = 0x01;
  constexpr uint8_t kTimeoutEnabled = 0x01;
  return static_cast<uint8_t>((motor_id & 0x0F) |
                              ((kEnabledStatus & 0x07) << 4) |
                              ((kTimeoutEnabled & 0x01) << 7));
}

}  // namespace

class HandControl {
 public:
  explicit HandControl(const std::string& hand_action_channel) {
    // A zero target is the configured open pose. Initialize it before either
    // worker starts so startup cannot race with an uninitialized command.
    target_.fill(0.0F);

    unitree::robot::ChannelFactory::Instance()->Init(0);
    left_publisher_.reset(new unitree::robot::ChannelPublisher<
                          unitree_hg::msg::dds_::HandCmd_>(kLeftCommandTopic));
    right_publisher_.reset(new unitree::robot::ChannelPublisher<
                           unitree_hg::msg::dds_::HandCmd_>(kRightCommandTopic));
    left_publisher_->InitChannel();
    right_publisher_->InitChannel();
    InitializeCommand(left_command_);
    InitializeCommand(right_command_);

    lcm_.subscribe(hand_action_channel, &HandControl::HandleHandAction, this);
    std::cout << "Dex3 hand control subscribed to LCM channel: "
              << hand_action_channel << '\n';
    lcm_thread_ = std::thread(&HandControl::LcmLoop, this);
    control_thread_ = std::thread(&HandControl::ControlLoop, this);
  }

 private:
  using HandCommand = unitree_hg::msg::dds_::HandCmd_;

  static void InitializeCommand(HandCommand& command) {
    command.motor_cmd().resize(kMotorCount);
    for (std::size_t motor_id = 0; motor_id < kMotorCount; ++motor_id) {
      auto& motor = command.motor_cmd()[motor_id];
      motor.mode(MakeMotorMode(motor_id));
      motor.q(0.0F);
      motor.dq(0.0F);
      motor.tau(0.0F);
      motor.kp(0.5F);
      motor.kd(0.1F);
    }
  }

  void HandleHandAction(const lcm::ReceiveBuffer*, const std::string&,
                        const hand_action_lcmt* message) {
    std::array<float, 2 * kMotorCount> candidate{};
    for (std::size_t index = 0; index < candidate.size(); ++index) {
      if (!std::isfinite(message->act[index])) {
        std::cerr << "[Warning] Ignore hand_action containing NaN/Inf\n";
        return;
      }
      candidate[index] = static_cast<float>(message->act[index]);
    }

    for (std::size_t index = 0; index < kMotorCount; ++index) {
      candidate[index] =
          std::clamp(candidate[index], kLeftMin[index], kLeftMax[index]);
      candidate[index + kMotorCount] = std::clamp(
          candidate[index + kMotorCount], kRightMin[index], kRightMax[index]);
    }

    std::lock_guard<std::mutex> lock(target_mutex_);
    target_ = candidate;
  }

  void LcmLoop() {
    while (true) {
      lcm_.handle();
    }
  }

  void WriteHand(HandCommand& command,
                 const std::array<float, 2 * kMotorCount>& target,
                 bool left) {
    const std::size_t offset = left ? 0 : kMotorCount;
    for (std::size_t index = 0; index < kMotorCount; ++index) {
      command.motor_cmd()[index].q(target[offset + index]);
    }
    if (left) {
      left_publisher_->Write(command);
    } else {
      right_publisher_->Write(command);
    }
  }

  void ControlLoop() {
    auto next_tick = std::chrono::steady_clock::now();
    while (true) {
      next_tick += kControlPeriod;
      std::array<float, 2 * kMotorCount> target_snapshot{};
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        target_snapshot = target_;
      }
      WriteHand(left_command_, target_snapshot, true);
      WriteHand(right_command_, target_snapshot, false);
      std::this_thread::sleep_until(next_tick);
    }
  }

  lcm::LCM lcm_;
  std::thread lcm_thread_;
  std::thread control_thread_;
  std::mutex target_mutex_;
  std::array<float, 2 * kMotorCount> target_{};

  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::HandCmd_>
      left_publisher_;
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::HandCmd_>
      right_publisher_;
  HandCommand left_command_;
  HandCommand right_command_;
};

int main(int argc, char** argv) {
  const std::string hand_action_channel =
      argc > 1 ? argv[1] : kDefaultHandActionChannel;
  HandControl hand_control(hand_action_channel);
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  return 0;
}
