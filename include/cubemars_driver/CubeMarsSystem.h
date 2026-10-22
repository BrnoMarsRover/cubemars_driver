#ifndef CUBEMARS_DRIVER_CUBEMARSSYSTEM_H
#define CUBEMARS_DRIVER_CUBEMARSSYSTEM_H

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/clock.hpp>

#include <cubemars_driver/CubeMars.h>

namespace cubemars_driver {

    /// ros2_control system of CubeMars AK actuators on one CAN bus (servo mode).
    ///
    /// The motors always run in velocity mode. A joint with a velocity command
    /// interface passes it through; a joint with a position command interface
    /// gets a PI loop here (position error -> velocity, position_kp, position_ki).
    /// States: position and velocity at the output shaft.
    ///
    /// Hardware parameters: can_interface, position_kp, position_ki.
    /// Joint parameters (as the cubemars_node config): can_id, kt, pole_pairs,
    /// gear_ratio, enc_off, direction, max_acceleration, position_feedback,
    /// min_position, max_position.
    class CubeMarsSystem : public hardware_interface::SystemInterface {
    public:
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        struct Joint {
            bool positionCommand = false;
            double position = 0.0;// [rad]
            double velocity = 0.0;// [rad/s]
            double command = 0.0; // [rad] or [rad/s]
            double integral = 0.0;// [rad s] of the position error
        };

        std::string bus_;
        double kp_ = 1.6;// [1/s]
        double ki_ = 0.1;// [1/s^2]
        std::vector<CubeMarsCommon::MotorConfig> configs_;
        std::vector<Joint> joints_;
        std::unique_ptr<CubeMars> driver_;
        rclcpp::Clock clock_{RCL_STEADY_TIME};// for throttled fault logs
    };

}// namespace cubemars_driver

#endif//CUBEMARS_DRIVER_CUBEMARSSYSTEM_H
