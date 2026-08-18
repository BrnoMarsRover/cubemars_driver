#ifndef CUBEMARS_DRIVER_CUBEMARS_H
#define CUBEMARS_DRIVER_CUBEMARS_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <cubemars_driver/CubeMarsCommon.h>

/// Low-level driver for CubeMars AK series actuators over SocketCAN.
/// Uses extended CAN frames (29-bit identifiers) as required by the CubeMars servo protocol.
class CubeMars {
public:
    /// Construct driver for one or more motors on the given CAN interface.
    explicit CubeMars(const std::string &canInterface, const std::vector<CubeMarsCommon::MotorConfig> &motors);
    ~CubeMars();

    CubeMars(const CubeMars &) = delete;
    CubeMars &operator=(const CubeMars &) = delete;

    /// Read all pending CAN feedback frames. Call this periodically.
    void readFeedback();

    /// Send a velocity command (rad/s at output shaft) for a motor by index.
    void setVelocity(size_t motorIndex, double radPerSec);

    /// Send a position command (rad at output shaft) for a motor by index.
    /// In PositionSpeed mode, the motor respects posVelLimit/posAccLimit from config.
    void setPosition(size_t motorIndex, double rad);

    /// Send a current (effort) command (Nm at output shaft) for a motor by index.
    void setCurrent(size_t motorIndex, double torqueNm);

    /// Send zero command to all motors (stop).
    void stopAll();

    /// Apply acceleration ramp and send the appropriate command for each motor.
    /// Call this at a fixed rate (e.g. 100 Hz). Uses the last set target per motor.
    void updateCommands(double dt);

    // --- Getters (thread-safe) ---

    [[nodiscard]] CubeMarsCommon::MotorState getState(size_t motorIndex) const;
    [[nodiscard]] size_t motorCount() const { return motors_.size(); }
    [[nodiscard]] const CubeMarsCommon::MotorConfig &motorConfig(size_t motorIndex) const { return motors_[motorIndex]->config; }

private:
    struct MotorRuntime {
        CubeMarsCommon::MotorConfig config;
        double erpmConversion = 0.0;
        /// Multiplier turning a reported angle into an output-shaft angle:
        /// 1.0 for output-mounted encoders, 1/gearRatio when measured at the
        /// rotor. Position commands apply its inverse.
        double positionScale = 1.0;

        // Targets (set by user commands)
        double targetVelocity = 0.0;
        double targetPosition = 0.0;
        double targetEffort = 0.0;

        // Ramped velocity (for acceleration limiting)
        double rampedVelocity = 0.0;

        // Feedback state
        mutable std::mutex stateMutex;
        CubeMarsCommon::MotorState state{};
    };

    bool canWrite(uint32_t id, const uint8_t *data, uint8_t len);
    bool canReadNonBlocking(uint32_t &id, uint8_t *data, uint8_t &len);

    int socket_ = -1;
    std::vector<std::unique_ptr<MotorRuntime>> motors_;
};

#endif// CUBEMARS_DRIVER_CUBEMARS_H
