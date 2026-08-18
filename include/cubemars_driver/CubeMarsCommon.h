#ifndef CUBEMARS_DRIVER_CUBEMARSCOMMON_H
#define CUBEMARS_DRIVER_CUBEMARSCOMMON_H

#include <cstdint>
#include <stdexcept>
#include <string>

namespace CubeMarsCommon {

enum class ControlMode : uint8_t {
    Current = 1,
    Speed = 3,
    Position = 4,
    SetOrigin = 5,
    PositionSpeed = 6,
};

enum class Fault : uint8_t {
    None = 0,
    MotorOverTemperature = 1,
    OverCurrent = 2,
    OverVoltage = 3,
    UnderVoltage = 4,
    Encoder = 5,
    MosfetOverTemperature = 6,
    MotorStall = 7,
};

inline std::string faultToString(Fault fault) {
    switch (fault) {
    case Fault::None: return "None";
    case Fault::MotorOverTemperature: return "Motor over-temperature";
    case Fault::OverCurrent: return "Over-current";
    case Fault::OverVoltage: return "Over-voltage";
    case Fault::UnderVoltage: return "Under-voltage";
    case Fault::Encoder: return "Encoder fault";
    case Fault::MosfetOverTemperature: return "MOSFET over-temperature";
    case Fault::MotorStall: return "Motor stall";
    default: return "Unknown fault";
    }
}

/// Where the motor's reported position is measured, which decides whether the
/// gear reduction has already been applied to it.
///
/// Output: the actuator has an encoder on the output shaft (e.g. AK60-39), so
///         the reported angle is already output degrees and must NOT be divided
///         by gearRatio. Such encoders are typically single-turn and lose their
///         reference across a power cycle.
/// Rotor:  no output encoder (e.g. AK40-10); position is measured before the
///         reduction, so it must be divided by gearRatio to get the output angle.
enum class PositionFeedback : uint8_t {
    Output = 0,
    Rotor = 1,
};

inline ControlMode controlModeFromString(const std::string &mode) {
    if (mode == "current") return ControlMode::Current;
    if (mode == "speed" || mode == "velocity") return ControlMode::Speed;
    if (mode == "position") return ControlMode::Position;
    if (mode == "position_speed") return ControlMode::PositionSpeed;
    throw std::runtime_error("Unknown CubeMars control mode: " + mode);
}

inline PositionFeedback positionFeedbackFromString(const std::string &src) {
    if (src == "output") return PositionFeedback::Output;
    if (src == "rotor") return PositionFeedback::Rotor;
    throw std::runtime_error("Unknown CubeMars position feedback source: " + src);
}

struct MotorConfig {
    std::string name;
    uint32_t canId = 0;
    double kt = 0.0;
    int polePairs = 0;
    int gearRatio = 0;
    double encoderOffset = 0.0;
    double torqueLimit = 0.0;
    double maxAcceleration = 0.0;
    int16_t posVelLimit = 0;
    int16_t posAccLimit = 0;
    bool readOnly = false;
    ControlMode controlMode = ControlMode::Speed;
    PositionFeedback positionFeedback = PositionFeedback::Output;
};

struct MotorState {
    double position = 0.0;    // rad (output shaft)
    double velocity = 0.0;    // rad/s (output shaft)
    double effort = 0.0;      // Nm (output shaft)
    int8_t temperature = 0;   // degrees C
    Fault fault = Fault::None;
};

}// namespace CubeMarsCommon

#endif// CUBEMARS_DRIVER_CUBEMARSCOMMON_H
