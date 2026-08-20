#include <cubemars_driver/CubeMars.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

CubeMars::CubeMars(const std::string &canInterface,
                   const std::vector<CubeMarsCommon::MotorConfig> &motors) {
    // Open raw CAN socket
    socket_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_ < 0) throw std::runtime_error("CubeMars: failed to open CAN socket");

    // Bind to interface
    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, canInterface.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &ifr) < 0) {
        ::close(socket_);
        throw std::runtime_error("CubeMars: CAN interface '" + canInterface + "' not found");
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(socket_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        ::close(socket_);
        throw std::runtime_error("CubeMars: failed to bind CAN socket");
    }

    // Set up receive filters for motor feedback (extended frames, match lower byte)
    std::vector<struct can_filter> filters(motors.size());
    for (size_t i = 0; i < motors.size(); ++i) {
        filters[i].can_id = motors[i].canId | CAN_EFF_FLAG;
        filters[i].can_mask = 0xFFU | CAN_EFF_FLAG;
    }
    ::setsockopt(socket_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                 static_cast<socklen_t>(filters.size() * sizeof(struct can_filter)));

    // Verify socket works with a test write. Non-blocking like every other send
    // here; on a freshly opened socket the queue is empty, so this still fails
    // only for a genuinely unusable interface (down, or gone) -- which is what it
    // is here to catch, and which the caller turns into a restart.
    struct can_frame testFrame {};
    testFrame.can_id = CAN_EFF_FLAG;
    testFrame.len = 0;
    if (::send(socket_, &testFrame, sizeof(testFrame), MSG_DONTWAIT) != static_cast<ssize_t>(sizeof(testFrame))) {
        ::close(socket_);
        throw std::runtime_error("CubeMars: CAN test write failed on '" + canInterface + "'");
    }

    // Initialize motor runtime data
    for (const auto &cfg : motors) {
        auto rt = std::make_unique<MotorRuntime>();
        rt->config = cfg;
        rt->erpmConversion = cfg.polePairs * cfg.gearRatio * 60.0 / (2.0 * M_PI);
        // Actuators with an output-shaft encoder already report post-reduction
        // angles; ones measuring at the rotor need the reduction divided out.
        rt->positionScale = (cfg.positionFeedback == CubeMarsCommon::PositionFeedback::Rotor &&
                             cfg.gearRatio > 0)
                                    ? 1.0 / static_cast<double>(cfg.gearRatio)
                                    : 1.0;
        motors_.push_back(std::move(rt));
    }
}

CubeMars::~CubeMars() {
    stopAll();
    if (socket_ >= 0) ::close(socket_);
}

bool CubeMars::canWrite(uint32_t id, const uint8_t *data, uint8_t len) {
    struct can_frame frame {};
    frame.can_id = id | CAN_EFF_FLAG;
    frame.len = len;
    if (data && len > 0) std::memcpy(frame.data, data, len);

    // MSG_DONTWAIT -- never a blocking write.
    //
    // CAN requires another node to ACK. With nothing else powered on the bus the
    // controller retries every frame indefinitely, the socket's transmit queue
    // fills, and a blocking write then parks the caller in the kernel forever.
    // Because this is called from the same timer callback that reads feedback and
    // publishes state, that stalls the ENTIRE node: no feedback, no topic, no
    // error -- it just goes quiet while looking alive. (Observed on the NUC: the
    // process sat in sock_alloc_send_pskb at 1.5% CPU publishing nothing.)
    //
    // Dropping the frame is strictly better than blocking: the next cycle
    // produces a fresher command anyway, so a stale one is worth nothing.
    if (::send(socket_, &frame, sizeof(frame), MSG_DONTWAIT) != static_cast<ssize_t>(sizeof(frame))) {
        ++txDropped_;
        return false;
    }
    return true;
}

bool CubeMars::canReadNonBlocking(uint32_t &id, uint8_t *data, uint8_t &len) {
    struct can_frame frame {};
    if (::recv(socket_, &frame, sizeof(frame), MSG_DONTWAIT) < 0) return false;
    std::memcpy(data, frame.data, frame.len);
    id = frame.can_id & (0xFFU | CAN_EFF_FLAG);
    len = frame.len;
    return true;
}

void CubeMars::readFeedback() {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;

    while (canReadNonBlocking(id, data, len)) {
        for (auto &motor : motors_) {
            if ((id & 0xFF) != motor->config.canId) continue;

            const std::lock_guard<std::mutex> lock(motor->stateMutex);
            auto &st = motor->state;

            auto posInt = static_cast<int16_t>(data[0] << 8 | data[1]);
            auto spdInt = static_cast<int16_t>(data[2] << 8 | data[3]);
            auto curInt = static_cast<int16_t>(data[4] << 8 | data[5]);

            // `direction` is applied last, to the finished output-shaft quantities,
            // so it flips the joint as ROS sees it without disturbing the unit
            // conversions above. The command path applies the same factor, which is
            // why direction is constrained to +-1 (see validateDirection).
            st.position = (posInt * 0.1 * M_PI / 180.0 * motor->positionScale - motor->config.encoderOffset) *
                          motor->config.direction;
            st.velocity = (spdInt * 10.0 / motor->erpmConversion) * motor->config.direction;
            st.effort = (curInt * 0.01 * motor->config.kt * motor->config.gearRatio) * motor->config.direction;
            st.temperature = static_cast<int8_t>(data[6]);
            st.fault = static_cast<CubeMarsCommon::Fault>(data[7]);
            break;
        }
    }
}

void CubeMars::setVelocity(size_t motorIndex, double radPerSec) {
    motors_.at(motorIndex)->targetVelocity = radPerSec * motors_.at(motorIndex)->config.direction;
}

void CubeMars::setPosition(size_t motorIndex, double rad) {
    motors_.at(motorIndex)->targetPosition = rad * motors_.at(motorIndex)->config.direction;
}

void CubeMars::setCurrent(size_t motorIndex, double torqueNm) {
    motors_.at(motorIndex)->targetEffort = torqueNm * motors_.at(motorIndex)->config.direction;
}

void CubeMars::stopAll() {
    for (auto &motor : motors_) {
        if (motor->config.readOnly) continue;
        motor->targetVelocity = 0.0;
        motor->targetEffort = 0.0;
        motor->rampedVelocity = 0.0;
        uint8_t data[8] = {};
        canWrite(motor->config.canId | static_cast<uint8_t>(CubeMarsCommon::ControlMode::Speed) << 8, data, 8);
    }
}

void CubeMars::updateCommands(double dt) {
    for (auto &motor : motors_) {
        if (motor->config.readOnly) continue;

        auto mode = motor->config.controlMode;
        uint8_t data[8] = {};

        // Soft travel limits, applied to the TARGET before it is encoded. Position
        // targets are clamped into range; velocity and current targets are refused
        // when they would drive further past a limit already reached.
        //
        // For velocity we also cap the speed to what can still be braked inside the
        // remaining travel (v <= sqrt(2*a*d)). Without that the joint stops only
        // once the limit is crossed and then coasts past it while the acceleration
        // ramp brings it down -- a limit you overshoot is not a limit.
        double posNow = 0.0;
        const bool limited = CubeMarsCommon::hasPositionLimits(motor->config);
        if (limited) {
            const std::lock_guard<std::mutex> lock(motor->stateMutex);
            posNow = motor->state.position;
        }

        switch (mode) {
        case CubeMarsCommon::ControlMode::Speed: {
            double target = motor->targetVelocity;
            if (motor->config.maxAcceleration > 0.0) {
                double maxDelta = motor->config.maxAcceleration * dt;
                double diff = target - motor->rampedVelocity;
                motor->rampedVelocity += std::clamp(diff, -maxDelta, maxDelta);
                target = motor->rampedVelocity;
            }
            if (limited) {
                const auto &cfg = motor->config;
                if (target > 0.0 && posNow >= cfg.maxPosition) target = 0.0;
                else if (target < 0.0 && posNow <= cfg.minPosition) target = 0.0;
                else if (cfg.maxAcceleration > 0.0 && target != 0.0) {
                    const double room = target > 0.0 ? cfg.maxPosition - posNow : posNow - cfg.minPosition;
                    const double vCap = std::sqrt(2.0 * cfg.maxAcceleration * std::max(room, 0.0));
                    target = std::clamp(target, -vCap, vCap);
                }
                motor->rampedVelocity = target;
            }
            auto speed = static_cast<int32_t>(target * motor->erpmConversion);
            if (std::abs(speed) >= 100000) break;
            data[0] = speed >> 24;
            data[1] = speed >> 16;
            data[2] = speed >> 8;
            data[3] = speed;
            canWrite(motor->config.canId | static_cast<uint8_t>(mode) << 8, data, 8);
            break;
        }
        case CubeMarsCommon::ControlMode::Current: {
            double effort = motor->targetEffort;
            if (limited) {
                if (effort > 0.0 && posNow >= motor->config.maxPosition) effort = 0.0;
                else if (effort < 0.0 && posNow <= motor->config.minPosition) effort = 0.0;
            }
            auto current = static_cast<int32_t>(effort * 1000.0 / motor->config.kt);
            if (std::abs(current) >= 60000) break;
            data[0] = current >> 24;
            data[1] = current >> 16;
            data[2] = current >> 8;
            data[3] = current;
            canWrite(motor->config.canId | static_cast<uint8_t>(mode) << 8, data, 8);
            break;
        }
        case CubeMarsCommon::ControlMode::Position: {
            double wanted = motor->targetPosition;
            if (limited) wanted = std::clamp(wanted, motor->config.minPosition, motor->config.maxPosition);
            auto position =
                static_cast<int32_t>((wanted + motor->config.encoderOffset) / motor->positionScale *
                                     10000.0 * 180.0 / M_PI);
            if (std::abs(position) >= 360000000) break;
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;
            canWrite(motor->config.canId | static_cast<uint8_t>(mode) << 8, data, 8);
            break;
        }
        case CubeMarsCommon::ControlMode::PositionSpeed: {
            double wanted = motor->targetPosition;
            if (limited) wanted = std::clamp(wanted, motor->config.minPosition, motor->config.maxPosition);
            auto position =
                static_cast<int32_t>((wanted + motor->config.encoderOffset) / motor->positionScale *
                                     10000.0 * 180.0 / M_PI);
            if (std::abs(position) >= 360000000) break;
            data[0] = position >> 24;
            data[1] = position >> 16;
            data[2] = position >> 8;
            data[3] = position;
            data[4] = motor->config.posVelLimit >> 8;
            data[5] = motor->config.posVelLimit;
            data[6] = motor->config.posAccLimit >> 8;
            data[7] = motor->config.posAccLimit;
            canWrite(motor->config.canId | static_cast<uint8_t>(mode) << 8, data, 8);
            break;
        }
        default: break;
        }
    }
}

CubeMarsCommon::MotorState CubeMars::getState(size_t motorIndex) const {
    const auto &motor = motors_.at(motorIndex);
    const std::lock_guard<std::mutex> lock(motor->stateMutex);
    return motor->state;
}
