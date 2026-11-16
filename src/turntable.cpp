#ifndef TURNTABLE_CPP
#define TURNTABLE_CPP

#include "globals.hpp"
#include "pin_colour.hpp"

#include <algorithm>
#include <cmath>

namespace TurntableConfig {
    constexpr double KP = 2.0;
    constexpr double KI = 0.0;
    constexpr double KD = 0.0;
    constexpr double SETTLE_DEGREES = 1.0;
}

class Turntable {
private:
    lemlib::PID positionPID{
        TurntableConfig::KP,
        TurntableConfig::KI,
        TurntableConfig::KD,
        0
    };
    double targetAngle = 0.0;
    bool positioning = false;

public:
    Turntable() {
        turntableMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
        turntableMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    }

    void setTargetAngle(double degrees) {
        if (targetAngle != degrees) positionPID.reset();
        targetAngle = degrees;
        positioning = true;
    }

    double getPosition() const {
        return turntableMotor.get_position();
    }

    bool isSettled() const {
        return std::fabs(targetAngle - getPosition()) < TurntableConfig::SETTLE_DEGREES;
    }

    void stop() {
        positioning = false;
        positionPID.reset();
        turntableMotor.move(0);
    }

    void run() {
        if (!positioning) {
            turntableMotor.move(0);
            return;
        }
        if (isSettled()) {
            turntableMotor.move(0);
            return;
        }

        const double output = static_cast<double>(positionPID.update(targetAngle - getPosition()));
        turntableMotor.move(output);
    }
};

inline Turntable turntable;

#endif
