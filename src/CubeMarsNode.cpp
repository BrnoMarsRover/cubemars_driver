#include <cubemars_driver/CubeMarsNode.h>

#include <functional>

CubeMarsNode::CubeMarsNode() : Node("cubemars_node") {
    onInitialize();
}

void CubeMarsNode::onInitialize() {
    using namespace CubeMarsNodeParameters;

    // Global parameters
    declare_parameter<std::string>(canInterface, "can0");
    declare_parameter<double>(updateRate, 100.0);
    declare_parameter<double>(commandTimeoutMs, 100.0);
    declare_parameter<std::vector<std::string>>(joints, std::vector<std::string>{});

    auto canItf = get_parameter(canInterface).as_string();
    auto rate = get_parameter(updateRate).as_double();
    auto jointNames = get_parameter(joints).as_string_array();
    commandTimeoutSec_ = get_parameter(commandTimeoutMs).as_double() / 1000.0;
    dt_ = 1.0 / rate;

    if (jointNames.empty()) {
        RCLCPP_FATAL(get_logger(), "Parameter '%s' is empty or not set", joints);
        throw std::runtime_error("No joints configured");
    }

    // Per-joint parameters
    std::vector<CubeMarsCommon::MotorConfig> motorConfigs;
    for (const auto &name : jointNames) {
        declare_parameter<int>(name + "." + canId);
        declare_parameter<double>(name + "." + kt);
        declare_parameter<int>(name + "." + polePairs);
        declare_parameter<int>(name + "." + gearRatio);
        declare_parameter<std::string>(name + "." + controlMode, "velocity");
        declare_parameter<double>(name + "." + encoderOffset, 0.0);
        declare_parameter<int>(name + "." + posVelLimit, 0);
        declare_parameter<int>(name + "." + posAccLimit, 0);
        declare_parameter<double>(name + "." + torqueLimit, 0.0);
        declare_parameter<double>(name + "." + maxAcceleration, 0.0);
        declare_parameter<bool>(name + "." + readOnly, false);
        declare_parameter<std::string>(name + "." + positionFeedback, "output");
        declare_parameter<double>(name + "." + direction, 1.0);

        CubeMarsCommon::MotorConfig cfg;
        cfg.name = name;
        cfg.canId = static_cast<uint32_t>(get_parameter(name + "." + canId).as_int());
        cfg.kt = get_parameter(name + "." + kt).as_double();
        cfg.polePairs = get_parameter(name + "." + polePairs).as_int();
        cfg.gearRatio = get_parameter(name + "." + gearRatio).as_int();
        cfg.encoderOffset = get_parameter(name + "." + encoderOffset).as_double();
        cfg.torqueLimit = get_parameter(name + "." + torqueLimit).as_double();
        cfg.maxAcceleration = get_parameter(name + "." + maxAcceleration).as_double();
        cfg.posVelLimit = static_cast<int16_t>(get_parameter(name + "." + posVelLimit).as_int());
        cfg.posAccLimit = static_cast<int16_t>(get_parameter(name + "." + posAccLimit).as_int());
        cfg.readOnly = get_parameter(name + "." + readOnly).as_bool();
        cfg.positionFeedback = CubeMarsCommon::positionFeedbackFromString(
                get_parameter(name + "." + positionFeedback).as_string());
        cfg.direction = CubeMarsCommon::validateDirection(
                get_parameter(name + "." + direction).as_double(), name);

        auto modeStr = get_parameter(name + "." + controlMode).as_string();
        cfg.controlMode = CubeMarsCommon::controlModeFromString(modeStr);

        motorConfigs.push_back(cfg);
        RCLCPP_INFO(get_logger(),
                    "Joint '%s': CAN ID %u, mode %s, kt %.4f, gear %d, position from %s, direction %+.0f",
                    name.c_str(), cfg.canId, modeStr.c_str(), cfg.kt, cfg.gearRatio,
                    get_parameter(name + "." + positionFeedback).as_string().c_str(), cfg.direction);
    }

    // Create driver
    canInterface_ = canItf;
    driver_ = std::make_unique<CubeMars>(canItf, motorConfigs);
    RCLCPP_INFO(get_logger(), "CAN communication active on '%s'", canItf.c_str());

    // Publishers and subscribers
    statePublisher_ = create_publisher<sensor_msgs::msg::JointState>("~/state", 10);
    commandSubscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "~/command", 10, std::bind(&CubeMarsNode::commandCallback, this, std::placeholders::_1));

    // Timer
    updateTimer_ = create_wall_timer(std::chrono::duration<double>(dt_),
                                     std::bind(&CubeMarsNode::updateTimerCallback, this));

    lastCommandTime_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(get_logger(), "CubeMars node started with %zu joints at %.1f Hz", motorConfigs.size(), rate);
}

void CubeMarsNode::commandCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    bool matched = false;

    for (size_t m = 0; m < msg->name.size(); ++m) {
        for (size_t j = 0; j < driver_->motorCount(); ++j) {
            if (msg->name[m] != driver_->motorConfig(j).name) continue;

            auto mode = driver_->motorConfig(j).controlMode;
            if (mode == CubeMarsCommon::ControlMode::Speed && m < msg->velocity.size()) {
                driver_->setVelocity(j, msg->velocity[m]);
                matched = true;
            } else if ((mode == CubeMarsCommon::ControlMode::Position ||
                        mode == CubeMarsCommon::ControlMode::PositionSpeed) &&
                       m < msg->position.size()) {
                driver_->setPosition(j, msg->position[m]);
                matched = true;
            } else if (mode == CubeMarsCommon::ControlMode::Current && m < msg->effort.size()) {
                driver_->setCurrent(j, msg->effort[m]);
                matched = true;
            }
            break;
        }
    }

    // Only a message that actually addressed one of OUR joints, with the field
    // its control mode needs, counts as command input. A JointState naming other
    // joints must not keep this driver's watchdog alive -- otherwise unrelated
    // traffic on a shared command topic silently disables the timeout.
    if (!matched) return;

    lastCommandTime_ = std::chrono::steady_clock::now();
    if (commandTimedOut_) {
        commandTimedOut_ = false;
        RCLCPP_INFO(get_logger(), "Command input resumed");
    }
}

void CubeMarsNode::updateTimerCallback() {
    // Command timeout check
    auto elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - lastCommandTime_).count();
    if (!commandTimedOut_ && elapsed > commandTimeoutSec_) {
        commandTimedOut_ = true;
        RCLCPP_WARN(get_logger(), "Command timeout (%.0f ms), stopping motors", elapsed * 1000.0);
        driver_->stopAll();
    }

    // Read feedback
    driver_->readFeedback();

    // Log faults
    for (size_t i = 0; i < driver_->motorCount(); ++i) {
        auto state = driver_->getState(i);
        const auto &cfg = driver_->motorConfig(i);
        if (state.fault != CubeMarsCommon::Fault::None) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Joint '%s' (CAN %u): %s",
                                  cfg.name.c_str(), cfg.canId,
                                  CubeMarsCommon::faultToString(state.fault).c_str());
        }
        if (cfg.torqueLimit > 0.0 && std::abs(state.effort) > cfg.torqueLimit) {
            RCLCPP_ERROR(get_logger(), "Joint '%s': torque limit exceeded (%.2f > %.2f)", cfg.name.c_str(),
                         state.effort, cfg.torqueLimit);
            driver_->stopAll();
        }
    }

    // Send commands
    driver_->updateCommands(dt_);

    // Transmit trouble is reported, not swallowed. Frames are dropped rather than
    // blocking (see CubeMars::canWrite), so without this the node would keep
    // publishing a perfectly healthy-looking state topic while nothing it sends
    // ever reaches a motor.
    const auto dropped = driver_->txDropped();
    if (dropped > txDroppedReported_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                             "CAN transmit failing on '%s': %lu frames dropped. The bus is not "
                             "draining - is anything else on it powered?",
                             canInterface_.c_str(), static_cast<unsigned long>(dropped));
        txDroppedReported_ = dropped;
    }

    // Publish state
    auto stateMsg = sensor_msgs::msg::JointState();
    stateMsg.header.stamp = now();
    stateMsg.name.resize(driver_->motorCount());
    stateMsg.position.resize(driver_->motorCount());
    stateMsg.velocity.resize(driver_->motorCount());
    stateMsg.effort.resize(driver_->motorCount());

    for (size_t i = 0; i < driver_->motorCount(); ++i) {
        auto state = driver_->getState(i);
        stateMsg.name[i] = driver_->motorConfig(i).name;
        stateMsg.position[i] = state.position;
        stateMsg.velocity[i] = state.velocity;
        stateMsg.effort[i] = state.effort;
    }
    statePublisher_->publish(stateMsg);
}
