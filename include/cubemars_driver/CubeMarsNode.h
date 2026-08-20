#ifndef CUBEMARS_DRIVER_CUBEMARSNODE_H
#define CUBEMARS_DRIVER_CUBEMARSNODE_H

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <cubemars_driver/CubeMars.h>

namespace CubeMarsNodeParameters {
static constexpr auto canInterface = "can_interface";
static constexpr auto updateRate = "update_rate";
static constexpr auto joints = "joints";
static constexpr auto commandTimeoutMs = "command_timeout_ms";

// Per-joint parameters (prefixed with joint name)
static constexpr auto canId = "can_id";
static constexpr auto kt = "kt";
static constexpr auto polePairs = "pole_pairs";
static constexpr auto gearRatio = "gear_ratio";
static constexpr auto controlMode = "control_mode";
static constexpr auto encoderOffset = "enc_off";
static constexpr auto posVelLimit = "pos_vel_limit";
static constexpr auto posAccLimit = "pos_acc_limit";
static constexpr auto torqueLimit = "trq_limit";
static constexpr auto maxAcceleration = "max_acceleration";
static constexpr auto readOnly = "read_only";
static constexpr auto positionFeedback = "position_feedback";
static constexpr auto direction = "direction";
static constexpr auto minPosition = "min_position";
static constexpr auto maxPosition = "max_position";
}// namespace CubeMarsNodeParameters

class CubeMarsNode : public rclcpp::Node {
public:
    CubeMarsNode();

private:
    void onInitialize();
    void updateTimerCallback();
    void commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::unique_ptr<CubeMars> driver_;
    double dt_ = 0.01;
    double commandTimeoutSec_ = 0.1;
    std::string canInterface_;
    /// Highest txDropped() already reported, so the warning fires on new drops
    /// rather than once per cycle forever.
    uint64_t txDroppedReported_ = 0;

    // Command timeout tracking
    std::chrono::time_point<std::chrono::steady_clock> lastCommandTime_;
    bool commandTimedOut_ = true;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr statePublisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr commandSubscription_;
    rclcpp::TimerBase::SharedPtr updateTimer_;
};

#endif// CUBEMARS_DRIVER_CUBEMARSNODE_H
