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
    intake.lowerPower = 0;
    intake.upperPower = 0;
    intake.rollerPower = 0;
    if(controller.get_digital(Buttons::L1)) {
        intake.lowerPower = 127;
        intake.upperPower = 90;
        intake.rollerPower = 60;
        scoreHeight = 1;
    } else if(controller.get_digital(Buttons::L2)) {
        intake.lowerPower = -110;
        intake.upperPower = -127;
    }
    if(controller.get_digital(Buttons::R1)) {
        intake.upperPower = 127;
        intake.rollerPower = 127;
        scoreHeight = 1;
    } else if(controller.get_digital(Buttons::R2)) {
        intake.upperPower = 127;
        intake.rollerPower = -127;
        scoreHeight = 2;
    } 
    int blockColour = checkColour();
    if (((allianceColour == 1 && blockColour == 2)|| (allianceColour == 2 && blockColour == 1)) && scoreHeight == 2) { //trying to score on middle goal
        ejectSpeed = 127;
        ejectState = true;
    } 
    else if (((allianceColour == 1 && blockColour == 2)|| (allianceColour == 2 && blockColour == 1)) && scoreHeight == 1) { //trying to score on long goal
        ejectSpeed = -127;
        ejectState = true;
    }
    if (ejectState) {
        ejectCounter++;
        intake.rollerPower = ejectSpeed;
        if (ejectCounter > 30) {
            ejectCounter = 0;
            ejectState = false;
        }
    }
    intake.moveLower(intake.lowerPower);
    intake.moveUpper(intake.upperPower);
    intake.moveScore(intake.rollerPower);
    scoreHeight = 0;
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


inline void activatePistons() {
    if (controller.get_digital_new_press(Buttons::X)) {
        leftWing.toggle();
        rightWing.toggle();
    }
    if (controller.get_digital_new_press(Buttons::A)) {
        tongue.toggle();
    }
}

inline void subsystemControls() {
    intakeControls(); 
    countBlocks();
    activatePistons();
    checkColour();
    int a = checkColour();
    pros::screen::print(TEXT_LARGE_CENTER, 4, "current: %i", lowerIntake.get_current_draw());
    pros::screen::print(TEXT_LARGE_CENTER, 1, "colour: %i", a);
    pros::screen::print(TEXT_LARGE_CENTER, 6, "distance: %i", intakeCountSensor.get_distance());
}

#endif