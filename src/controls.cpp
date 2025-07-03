#ifndef controls_cpp
#define controls_cpp

#include "globals.hpp"
#include "other_functions.cpp"
#include "intake.cpp"
#include "piston.cpp"

inline int checkColour() {
	if (colourSortSensor.get_hue() > 160 && colourSortSensor.get_hue() < 320 && colourSortSensor.get_proximity() > 160) {
		return 2; //blue
	}
	else if (colourSortSensor.get_hue() < 10 && colourSortSensor.get_proximity() > 160){
		return 1; //red
	} else {
        return 0; //background
    }
}

inline void intakeControls() {
    if(controller.get_digital(Buttons::L1)) {
        intake.setState(IntakeState::INTAKE);
    } else if(controller.get_digital(Buttons::L2)) {
        // scoreLow slows down the lower stage to fit 7 on low goal
        if(intakeLift.getState() == true) {
            intake.setState(IntakeState::SCORELOW);
        } else {
            intake.setState(IntakeState::OUTTAKE);
        }
    } else if(controller.get_digital(Buttons::R1)) {
        intake.setState(IntakeState::SCOREHIGH);
    } else if(controller.get_digital(Buttons::R2)) {
        intake.setState(IntakeState::SCOREMID);
    } else {
        intake.setState(IntakeState::IDLE);
    }

    // uses the same park button to exit off park
    if(controller.get_digital_new_press(Buttons::park)) {
        if(park.getState() == true) {
            park.setState(false);
        } else {
            intake.setState(IntakeState::PARK);
        }
    }
}


// inline void countBlocks() {
//     if (intakeCountSensor.get_distance() < 45) {
//         intake.sensorBlocked = true;
//     }
//     if (intake.sensorBlocked == true && intakeCountSensor.get_distance() > 100) {
//         intake.sensorBlocked = false;
//         intake.counter += 1;
//     }
// }


inline void activatePistons() {
    if (controller.get_digital_new_press(Buttons::wing)) {
        leftWing.toggle();
        rightWing.toggle();
    }
    if (controller.get_digital_new_press(Buttons::tongue)) {
        tongue.toggle();
    }
    // backup park button if the other doesn't work
    if(activatePark) {
        park.setState(true);
        activatePark = false;
    }
    if (controller.get_digital_new_press(Buttons::DOWN)) {
        park.toggle();
    }
    if (controller.get_digital_new_press(Buttons::intakeLift)) {
        intakeLift.toggle();
    }
}

inline void subsystemControls() {
    intakeControls(); 
    activatePistons();
    checkColour();
    int a = checkColour();
}

inline void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX, false, 0.5);
}

inline void displayData() {
    // display lemlib pose
    // lemlib::Pose pose = chassis.getPose();
    // pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "y: %f", pose.y);
    // pros::screen::print(TEXT_LARGE_CENTER, 5, "heading: %f", pose.theta);

    pros::screen::print(TEXT_LARGE_CENTER, 1, "state: %i", intake.returnState());
    pros::screen::print(TEXT_LARGE_CENTER, 3, "scoringCurent: %i", scoringRoller.get_current_draw());
    pros::screen::print(TEXT_LARGE_CENTER, 5, "upperSpeed: %f", upperIntake.get_actual_velocity());
}

#endif