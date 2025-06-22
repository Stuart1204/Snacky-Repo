#ifndef controls_cpp
#define controls_cpp

#include "globals.hpp"
#include "motor_functions.cpp"
#include "intake.cpp"
#include "piston.cpp"

inline void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX, false, 0.5);
}

inline void intakeControls() {
    intake.lowerPower = 0;
    intake.upperPower = 0;
    intake.rollerPower = 0;
    if(controller.get_digital(Buttons::L1)) {
        intake.lowerPower = 127;
        intake.upperPower = 90;
        intake.rollerPower = 60;
    } else if(controller.get_digital(Buttons::L2)) {
        intake.lowerPower = -110;
        intake.upperPower = -127;
    }
    if(controller.get_digital(Buttons::R1)) {
        intake.upperPower = 127;
        intake.rollerPower = 127;
    } else if(controller.get_digital(Buttons::R2)) {
        intake.upperPower = 127;
        intake.rollerPower = -127;
    } 
    intake.moveLower(intake.lowerPower);
    intake.moveUpper(intake.upperPower);
    intake.moveScore(intake.rollerPower);
}

inline void intakeFill() {
    if (controller.get_digital(Buttons::A)) {
        if (intake.counter <= 3) {
            intake.moveLower(127);
        }
        else if (intake.counter == 4) {
            if (intakeHandOffSensor.get_proximity() < 150) {
                intake.upperMoveVelocity(20);
            }
            intake.upperMoveVelocity(0);
        }

    }
}
inline void countBlocks() {
    if (intakeCountSensor.get_distance() < 45) {
        intake.sensorBlocked = true;
    }
    if (intake.sensorBlocked == true && intakeCountSensor.get_distance() > 100) {
        intake.sensorBlocked = false;
        intake.counter += 1;
    }
}
inline void activateWings() {
    if (controller.get_digital_new_press(Buttons::X)) {
        leftWing.toggle();
        rightWing.toggle();
    }
}

inline void subsystemControls() {
    intakeControls();
    countBlocks();
    activateWings();
    pros::screen::print(TEXT_LARGE_CENTER, 4, "current: %i", lowerIntake.get_current_draw());
    pros::screen::print(TEXT_LARGE_CENTER, 1, "counter: %i", intake.counter);
    pros::screen::print(TEXT_LARGE_CENTER, 6, "distance: %i", intakeCountSensor.get_distance());
}

#endif