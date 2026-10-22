#include <cubemars_driver/CubeMarsSystem.h>

#include <algorithm>
#include <cmath>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

namespace cubemars_driver {

    namespace {
        const rclcpp::Logger LOGGER = rclcpp::get_logger("CubeMarsSystem");

        /// |integral of the position error| stays below this [rad s], as in the asgard kinematics
        constexpr double MAX_INTEGRAL = 1.0;

        std::string param(const hardware_interface::ComponentInfo &joint, const std::string &name,
                          const std::string &fallback) {
            const auto it = joint.parameters.find(name);
            return it == joint.parameters.end() ? fallback : it->second;
        }

        CubeMarsCommon::MotorConfig motorConfig(const hardware_interface::ComponentInfo &joint) {
            CubeMarsCommon::MotorConfig cfg;
            cfg.name = joint.name;
            cfg.canId = static_cast<uint32_t>(std::stoul(joint.parameters.at("can_id")));
            cfg.kt = std::stod(joint.parameters.at("kt"));
            cfg.polePairs = std::stoi(joint.parameters.at("pole_pairs"));
            cfg.gearRatio = std::stoi(joint.parameters.at("gear_ratio"));
            cfg.encoderOffset = std::stod(param(joint, "enc_off", "0.0"));
            cfg.maxAcceleration = std::stod(param(joint, "max_acceleration", "0.0"));
            cfg.minPosition = std::stod(param(joint, "min_position", "0.0"));
            cfg.maxPosition = std::stod(param(joint, "max_position", "0.0"));
            cfg.direction = CubeMarsCommon::validateDirection(std::stod(param(joint, "direction", "1.0")), joint.name);
            cfg.positionFeedback = CubeMarsCommon::positionFeedbackFromString(param(joint, "position_feedback", "output"));
            cfg.controlMode = CubeMarsCommon::ControlMode::Speed;
            return cfg;
        }
    }// namespace

    hardware_interface::CallbackReturn CubeMarsSystem::on_init(const hardware_interface::HardwareInfo &info) {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

        try {
            const auto &params = info_.hardware_parameters;
            bus_ = params.at("can_interface");
            if (params.count("position_kp")) kp_ = std::stod(params.at("position_kp"));
            if (params.count("position_ki")) ki_ = std::stod(params.at("position_ki"));

            for (const auto &joint: info_.joints) {
                if (joint.command_interfaces.size() != 1)
                    throw std::invalid_argument(joint.name + " needs exactly one command interface");
                const auto &command = joint.command_interfaces[0].name;
                if (command != hardware_interface::HW_IF_POSITION && command != hardware_interface::HW_IF_VELOCITY)
                    throw std::invalid_argument(joint.name + ": command interface must be position or velocity");

                configs_.push_back(motorConfig(joint));
                joints_.push_back(Joint{.positionCommand = command == hardware_interface::HW_IF_POSITION});
            }
        } catch (const std::exception &e) {
            RCLCPP_FATAL(LOGGER, "%s: invalid configuration: %s", info_.name.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubeMarsSystem::on_configure(const rclcpp_lifecycle::State &) {
        try {
            driver_ = std::make_unique<CubeMars>(bus_, configs_);
        } catch (const std::exception &e) {
            RCLCPP_FATAL(LOGGER, "%s: %s", info_.name.c_str(), e.what());
            return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(LOGGER, "%s: %zu motors on '%s'", info_.name.c_str(), configs_.size(), bus_.c_str());
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubeMarsSystem::on_cleanup(const rclcpp_lifecycle::State &) {
        driver_.reset();
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubeMarsSystem::on_activate(const rclcpp_lifecycle::State &) {
        // start from where the joints are: no jump to a stale target
        driver_->readFeedback();
        for (size_t i = 0; i < joints_.size(); ++i) {
            auto &joint = joints_[i];
            joint.position = driver_->getState(i).position;
            joint.command = joint.positionCommand ? joint.position : 0.0;
            joint.integral = 0.0;
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CubeMarsSystem::on_deactivate(const rclcpp_lifecycle::State &) {
        driver_->stopAll();
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> CubeMarsSystem::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> interfaces;
        for (size_t i = 0; i < joints_.size(); ++i) {
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joints_[i].position);
            interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joints_[i].velocity);
        }
        return interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CubeMarsSystem::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> interfaces;
        for (size_t i = 0; i < joints_.size(); ++i)
            interfaces.emplace_back(info_.joints[i].name, info_.joints[i].command_interfaces[0].name, &joints_[i].command);
        return interfaces;
    }

    hardware_interface::return_type CubeMarsSystem::read(const rclcpp::Time &, const rclcpp::Duration &) {
        driver_->readFeedback();
        for (size_t i = 0; i < joints_.size(); ++i) {
            const auto state = driver_->getState(i);
            joints_[i].position = state.position;
            joints_[i].velocity = state.velocity;
            if (state.fault != CubeMarsCommon::Fault::None) {
                RCLCPP_ERROR_THROTTLE(LOGGER, clock_, 5000, "%s (CAN %u): %s", info_.joints[i].name.c_str(),
                                      configs_[i].canId, CubeMarsCommon::faultToString(state.fault).c_str());
            }
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CubeMarsSystem::write(const rclcpp::Time &, const rclcpp::Duration &period) {
        const double dt = period.seconds();
        for (size_t i = 0; i < joints_.size(); ++i) {
            auto &joint = joints_[i];
            double velocity = 0.0;
            if (!std::isfinite(joint.command)) {
                joint.integral = 0.0;
            } else if (joint.positionCommand) {
                const double error = joint.command - joint.position;
                joint.integral = std::clamp(joint.integral + error * dt, -MAX_INTEGRAL, MAX_INTEGRAL);
                velocity = kp_ * error + ki_ * joint.integral;
            } else {
                velocity = joint.command;
            }
            driver_->setVelocity(i, velocity);
        }
        driver_->updateCommands(dt);
        return hardware_interface::return_type::OK;
    }

}// namespace cubemars_driver

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(cubemars_driver::CubeMarsSystem, hardware_interface::SystemInterface)
