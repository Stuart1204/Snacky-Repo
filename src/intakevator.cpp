#ifndef INTAKEVATOR_CPP
#define INTAKEVATOR_CPP

#include "globals.hpp"
#include "piston.cpp"
#include "claw.cpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace IntakevatorConfig {
    constexpr bool WRIST_RETRACTED = false;
    constexpr bool WRIST_EXTENDED = true;

    constexpr double TICKS_PER_INCH = 83.0;
    constexpr double LIFT_MIN = -100.0;
    constexpr double LIFT_MAX = 100.0;
    constexpr double INTAKE_ELEV = 5.5;
    constexpr double SCORE_LIFT_DELTA_IN = 0.5;
    constexpr double RESET_ELEV = 7.0;
    constexpr double FAST_RESET_ELEV = 3.0;
    constexpr double RESET_ELEV_SEEK = -100.0;
    constexpr double LOAD_ELEV = 0.0;

    constexpr double PIN_0_ELEV = 0.5;
    constexpr double PIN_1_ELEV = 8.6;
    constexpr double PIN_2_ELEV = 15.6;
    constexpr double PIN_3_ELEV = 24.1;
    constexpr double PIN_4_ELEV = 32.6;
    constexpr double PIN_MAX_ELEV = 40.0;
    constexpr double PIN_HEIGHTS[] = {PIN_0_ELEV, PIN_1_ELEV, PIN_2_ELEV, PIN_3_ELEV, PIN_4_ELEV};
    constexpr int NUM_PIN_LEVELS = 5;
    constexpr double ALLIANCE_OFFSET = 3.5;

    constexpr double GROUND_PICKUP_ELEV = 5.0;
    constexpr double GROUND_CLAMP_ELEV = -4.5;

    constexpr double ELEV_KP = 45.0;
    constexpr double ELEV_KI = 0.0;
    constexpr double ELEV_KD = 0.0;
    constexpr double ELEV_SETTLE_IN = 1.0;

    constexpr double STALL_VELOCITY_THRESHOLD = 200.0;
    constexpr double STALL_CURRENT_THRESHOLD = 1200.0;
    constexpr uint32_t STALL_GRACE_PERIOD_MS = 150;
    constexpr uint32_t STALL_TIMEOUT_MS = 900;

    constexpr uint32_t GLOBAL_STALL_DURATION_MS = 500;
    constexpr double GLOBAL_STALL_VELOCITY = 20.0;
    constexpr double GLOBAL_STALL_CURRENT = 2000.0;

    constexpr uint32_t WRIST_EXTEND_TIME_MS = 300;
    constexpr uint32_t WRIST_RETRACT_TIME_MS = 300;
    constexpr uint32_t CLAW_ACTIVATE_DELAY_MS = 300;
    constexpr uint32_t PTO_SWITCH_DELAY_MS = 100;
    constexpr int SCORE_RESET_TRAVEL_CENTIDEG = 2500;

    constexpr int INTAKE_SPEED = 127;
    constexpr int OUTTAKE_SPEED = -127;
}

enum class IntakevatorState {
    IDLE,
    INTAKE,
    OUTTAKE,
    GRAB,
    GROUND_PICKUP,
    STOW,
    SCORE,
    RESET,
    MANUAL,
    HOLD,
    PIN_HEIGHT,
    ESTOP
};

class Intakevator {
private:
    lemlib::PID liftPID{
        IntakevatorConfig::ELEV_KP,
        IntakevatorConfig::ELEV_KI,
        IntakevatorConfig::ELEV_KD,
        0
    };

    IntakevatorState currentState = IntakevatorState::IDLE;
    IntakevatorState lastState = IntakevatorState::IDLE;
    int phase = 0;
    bool stateComplete = false;

    double liftTarget = 0.0;
    double tempLiftTarget = 0.0;
    double holdLiftTarget = 0.0;
    bool holdWristExtended = IntakevatorConfig::WRIST_RETRACTED;
    bool holdWristFirst = false;

    int maxSpeed = 127;
    int odomPosition = 0;
    int liftPower = 0;

    uint32_t motionStartTime = 0;
    uint32_t stallStartTime = 0;
    uint32_t wristLastChangeTime = 0;
    uint32_t globalStallStart = 0;
    bool globalStallActive = false;

    static double clampD(double value, double low, double high) {
        return std::clamp(value, low, high);
    }

    void stopMotors() {
        liftPower = 0;
        intakevatorMotors.move(0);
    }

    void selectPto(bool intakeSelected) {
        if (intakevatorPto.getState() == intakeSelected) return;
        stopMotors();
        intakevatorPto.setState(intakeSelected);
        pros::delay(IntakevatorConfig::PTO_SWITCH_DELAY_MS);
    }

    void runIntake(int speed) {
        liftPower = 0;
        globalStallActive = false;
        selectPto(true);
        intakevatorMotors.move(speed);
    }

    double getAverageVelocity() {
        return (std::fabs(intakevatorMotors.get_actual_velocity(0)) +
                std::fabs(intakevatorMotors.get_actual_velocity(1))) / 2.0;
    }

    double getAverageCurrent() {
        return (std::abs(intakevatorMotors.get_current_draw(0)) +
                std::abs(intakevatorMotors.get_current_draw(1))) / 2.0;
    }

    void setWrist(bool extended) {
        if (wristPiston.getState() != extended) {
            wristPiston.setState(extended);
            wristLastChangeTime = pros::millis();
        }
    }

    bool isWristReady() const {
        const uint32_t requiredTime = wristPiston.getState()
            ? IntakevatorConfig::WRIST_EXTEND_TIME_MS
            : IntakevatorConfig::WRIST_RETRACT_TIME_MS;
        return pros::millis() - wristLastChangeTime >= requiredTime;
    }

    bool isLiftSettled() {
        return std::fabs(getPosition() - liftTarget) < IntakevatorConfig::ELEV_SETTLE_IN;
    }

    bool isSettled() {
        return isLiftSettled() && isWristReady();
    }

    bool usesDirectIntakePower() {
        if (currentState == IntakevatorState::INTAKE ||
            currentState == IntakevatorState::OUTTAKE) {
            return true;
        }
        return currentState == IntakevatorState::GRAB &&
               lastState != IntakevatorState::GROUND_PICKUP && isLiftSettled();
    }

    bool isMotionStalled() {
        if (pros::millis() - motionStartTime < IntakevatorConfig::STALL_GRACE_PERIOD_MS) {
            stallStartTime = pros::millis();
            return false;
        }
        if (pros::millis() - motionStartTime > IntakevatorConfig::STALL_TIMEOUT_MS) return true;

        const bool stalledNow =
            getAverageVelocity() < IntakevatorConfig::STALL_VELOCITY_THRESHOLD &&
            getAverageCurrent() > IntakevatorConfig::STALL_CURRENT_THRESHOLD;
        if (!stalledNow) stallStartTime = pros::millis();
        return pros::millis() - stallStartTime > 90;
    }

    bool checkGlobalStallProtection() {
        if (currentState == IntakevatorState::IDLE ||
            currentState == IntakevatorState::ESTOP) {
            return false;
        }

        const bool stalledNow = std::abs(liftPower) > 70 &&
            getAverageVelocity() < IntakevatorConfig::GLOBAL_STALL_VELOCITY &&
            getAverageCurrent() > IntakevatorConfig::GLOBAL_STALL_CURRENT;

        if (!stalledNow) {
            globalStallActive = false;
            return false;
        }
        if (!globalStallActive) {
            globalStallActive = true;
            globalStallStart = pros::millis();
            return false;
        }
        return pros::millis() - globalStallStart > IntakevatorConfig::GLOBAL_STALL_DURATION_MS;
    }

    void beginMotion() {
        motionStartTime = pros::millis();
        stallStartTime = pros::millis();
        liftPID.reset();
    }

    void runLift() {
        if (isSettled() && stateComplete) {
            stopMotors();
            return;
        }

        const double output = clampD(
            liftPID.update(liftTarget - getPosition()), -maxSpeed, maxSpeed);
        liftPower = static_cast<int>(output);
        selectPto(false);
        intakevatorMotors.move(output);
    }

    void runHold() {
        switch (phase) {
            case 0:
                if (holdWristFirst) {
                    setWrist(holdWristExtended);
                    if (isWristReady()) phase = 1;
                } else {
                    liftTarget = holdLiftTarget;
                    if (isLiftSettled()) phase = 1;
                }
                break;
            case 1:
                if (holdWristFirst) {
                    liftTarget = holdLiftTarget;
                    if (isLiftSettled()) {
                        phase = 2;
                        stateComplete = true;
                    }
                } else {
                    setWrist(holdWristExtended);
                    if (isWristReady()) {
                        phase = 2;
                        stateComplete = true;
                    }
                }
                break;
            case 2:
                break;
        }
    }

    void runIntakePickup() {
        switch (phase) {
            case 0:
                liftTarget = IntakevatorConfig::INTAKE_ELEV;
                if (isLiftSettled()) phase = 1;
                break;
            case 1:
                setWrist(IntakevatorConfig::WRIST_RETRACTED);
                if (isWristReady()) {
                    phase = 2;
                    stateComplete = true;
                    liftTarget = getPosition();
                }
                break;
            case 2:
                claw.setState(ClawState::INTAKE_FULL);
                break;
        }
    }

    void runReset() {
        switch (phase) {
            case 0: {
                const double safeExtensionHeight = isFastReset
                    ? IntakevatorConfig::FAST_RESET_ELEV
                    : IntakevatorConfig::RESET_ELEV;
                if (wristPiston.getState() != IntakevatorConfig::WRIST_EXTENDED &&
                    getPosition() < safeExtensionHeight) {
                    liftTarget = safeExtensionHeight;
                    phase = 1;
                } else {
                    setWrist(IntakevatorConfig::WRIST_EXTENDED);
                    phase = 2;
                }
                break;
            }
            case 1:
                if (isLiftSettled()) {
                    setWrist(IntakevatorConfig::WRIST_EXTENDED);
                    phase = 2;
                }
                break;
            case 2:
                if (isWristReady()) {
                    liftTarget = IntakevatorConfig::RESET_ELEV_SEEK;
                    beginMotion();
                    phase = 3;
                }
                break;
            case 3:
                if (isMotionStalled()) {
                    stopMotors();
                    resetEncoders();
                    stateComplete = true;
                }
                break;
        }
    }

    void runGroundPickup() {
        switch (phase) {
            case 0:
                setWrist(IntakevatorConfig::WRIST_EXTENDED);
                if (isWristReady()) phase = 1;
                break;
            case 1:
                liftTarget = IntakevatorConfig::GROUND_PICKUP_ELEV;
                if (isLiftSettled() || isMotionStalled()) {
                    stateComplete = true;
                    liftTarget = getPosition();
                    phase = 2;
                }
                break;
            case 2:
                claw.setState(ClawState::INTAKE_FULL);
                setState(IntakevatorState::GRAB);
                break;
        }
    }

    void runGrab() {
        switch (phase) {
            case 0:
                if (claw.hasObject()) {
                    motionStartTime = pros::millis();
                    phase = 1;
                }
                break;
            case 1:
                if (pros::millis() - motionStartTime >= IntakevatorConfig::CLAW_ACTIVATE_DELAY_MS) {
                    stateComplete = true;
                    claw.setState(ClawState::INTAKE_SLOW);
                    goToPinHeight(1);
                }
                break;
        }
    }

    void runScore() {
        switch (phase) {
            case 0:
                claw.setState(ClawState::OUTTAKE_FULL);
                motionStartTime = pros::millis();
                phase = 1;
                break;
            case 1:
                if (pros::millis() - motionStartTime >= IntakevatorConfig::CLAW_ACTIVATE_DELAY_MS) {
                    claw.setState(ClawState::IDLE);
                    liftTarget = tempLiftTarget + IntakevatorConfig::SCORE_LIFT_DELTA_IN;
                    phase = 2;
                }
                break;
            case 2:
                if (isLiftSettled()) {
                    stateComplete = true;
                    if (isAutoReset &&
                        vertical_rotation.get_position() - odomPosition >
                            IntakevatorConfig::SCORE_RESET_TRAVEL_CENTIDEG) {
                        setState(IntakevatorState::RESET);
                    }
                }
                break;
        }
    }

    void runPinHeight() {
        if (isLiftSettled()) stateComplete = true;
    }

    double getPinElevation(int level) {
        if (level < 0 || level >= IntakevatorConfig::NUM_PIN_LEVELS) {
            return IntakevatorConfig::PIN_1_ELEV -
                   (isNeutralGoal ? 0.0 : IntakevatorConfig::ALLIANCE_OFFSET);
        }
        return IntakevatorConfig::PIN_HEIGHTS[level] -
               (isNeutralGoal ? 0.0 : IntakevatorConfig::ALLIANCE_OFFSET);
    }

public:
    Intakevator() {
        intakevatorMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
        intakevatorPto.setState(false);
        wristPiston.setState(IntakevatorConfig::WRIST_RETRACTED);
        wristLastChangeTime = pros::millis();
    }

    void setState(IntakevatorState state) {
        if (state == currentState) return;

        lastState = currentState;
        currentState = state;
        phase = 0;
        stateComplete = false;
        maxSpeed = 127;
        stopMotors();
        beginMotion();

        switch (currentState) {
            case IntakevatorState::IDLE:
            case IntakevatorState::ESTOP:
                claw.setState(ClawState::IDLE);
                break;
            case IntakevatorState::SCORE:
                odomPosition = vertical_rotation.get_position();
                tempLiftTarget = getPosition();
                clamp.setState(false);
                claw.setState(ClawState::OUTTAKE_FULL);
                break;
            case IntakevatorState::GRAB:
                clamp.setState(false);
                if (lastState != IntakevatorState::GROUND_PICKUP) {
                    setWrist(IntakevatorConfig::WRIST_RETRACTED);
                }
                break;
            case IntakevatorState::GROUND_PICKUP:
                clamp.setState(false);
                break;
            case IntakevatorState::MANUAL:
                liftTarget = getPosition();
                break;
            case IntakevatorState::INTAKE:
            case IntakevatorState::OUTTAKE:
            case IntakevatorState::STOW:
            case IntakevatorState::RESET:
            case IntakevatorState::HOLD:
            case IntakevatorState::PIN_HEIGHT:
                break;
        }
    }

    IntakevatorState getState() const {
        return currentState;
    }

    bool isStateComplete() const {
        return stateComplete;
    }

    void setMaxSpeed(int speed) {
        maxSpeed = speed;
    }

    double getPosition() {
        const double rightTicks = intakevatorMotors.get_position(0);
        const double leftTicks = intakevatorMotors.get_position(1);
        return (leftTicks + rightTicks) / 2.0 / IntakevatorConfig::TICKS_PER_INCH;
    }

    void goToPinHeight(int level) {
        level = std::clamp(level, 0, IntakevatorConfig::NUM_PIN_LEVELS - 1);
        liftTarget = clampD(getPinElevation(level), 0.0, IntakevatorConfig::PIN_MAX_ELEV);

        if (currentState != IntakevatorState::PIN_HEIGHT) {
            setState(IntakevatorState::PIN_HEIGHT);
        } else {
            phase = 0;
            stateComplete = false;
            beginMotion();
        }
    }

    void manualMove(int direction) {
        if (currentState != IntakevatorState::MANUAL &&
            currentState != IntakevatorState::PIN_HEIGHT) {
            setState(IntakevatorState::MANUAL);
        }
        liftTarget = direction > 0 ? IntakevatorConfig::LIFT_MAX : IntakevatorConfig::LIFT_MIN;
    }

    void manualStop() {
        stopMotors();
        liftTarget = getPosition();
        liftPID.reset();
    }

    void moveTo(double elevInches, bool wristExtended, bool wristFirst = false) {
        holdLiftTarget = elevInches;
        holdWristExtended = wristExtended;
        holdWristFirst = wristFirst;
        setState(IntakevatorState::HOLD);
    }

    void resetEncoders() {
        intakevatorMotors.tare_position_all();
        liftTarget = 0.0;
        liftPID.reset();
    }

    void waitUntilComplete(uint32_t timeoutMs = 5000) {
        const uint32_t start = pros::millis();
        while (!stateComplete && pros::millis() - start < timeoutMs) pros::delay(10);
    }

    void run() {
        if (!usesDirectIntakePower() && checkGlobalStallProtection()) {
            setState(IntakevatorState::ESTOP);
            return;
        }

        switch (currentState) {
            case IntakevatorState::IDLE:
            case IntakevatorState::ESTOP:
                claw.setState(ClawState::IDLE);
                stopMotors();
                return;
            case IntakevatorState::INTAKE:
                runIntake(IntakevatorConfig::INTAKE_SPEED);
                return;
            case IntakevatorState::OUTTAKE:
                runIntake(IntakevatorConfig::OUTTAKE_SPEED);
                return;
            case IntakevatorState::GRAB:
                runGrab();
                break;
            case IntakevatorState::GROUND_PICKUP:
                runGroundPickup();
                break;
            case IntakevatorState::STOW:
                runIntakePickup();
                break;
            case IntakevatorState::SCORE:
                runScore();
                break;
            case IntakevatorState::RESET:
                runReset();
                break;
            case IntakevatorState::MANUAL:
                runLift();
                return;
            case IntakevatorState::HOLD:
                runHold();
                break;
            case IntakevatorState::PIN_HEIGHT:
                runPinHeight();
                break;
        }

        if (usesDirectIntakePower()) {
            runIntake(IntakevatorConfig::INTAKE_SPEED);
        } else {
            runLift();
        }
    }
};

inline Intakevator intakevator;

#endif
