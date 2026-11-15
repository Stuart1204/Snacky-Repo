#ifndef CLAW_CPP
#define CLAW_CPP

#include "globals.hpp"

namespace ClawConfig {
    constexpr int FULL_SPEED = 127;
    constexpr int SLOW_INTAKE_SPEED = 60;
    constexpr int DETECT_DISTANCE_MM = 100;
}

enum class ClawState {
    IDLE,
    INTAKE_FULL,
    OUTTAKE_FULL,
    INTAKE_SLOW
};

class Claw {
private:
    ClawState currentState = ClawState::IDLE;

public:
    void setState(ClawState state) {
        currentState = state;
    }

    ClawState getState() const {
        return currentState;
    }

    bool hasObject() const {
        const int distanceMM = clawDistanceSensor.get_distance();
        return clawDistanceSensor.is_installed() && distanceMM >= 0 &&
               distanceMM < ClawConfig::DETECT_DISTANCE_MM;
    }

    void run() {
        switch (currentState) {
            case ClawState::IDLE:
                clawMotor.move(0);
                break;
            case ClawState::INTAKE_FULL:
                clawMotor.move(ClawConfig::FULL_SPEED);
                break;
            case ClawState::OUTTAKE_FULL:
                clawMotor.move(-ClawConfig::FULL_SPEED);
                break;
            case ClawState::INTAKE_SLOW:
                clawMotor.move(ClawConfig::SLOW_INTAKE_SPEED);
                break;
        }
    }
};

inline Claw claw;

#endif
