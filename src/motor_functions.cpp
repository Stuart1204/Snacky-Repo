#include "motor_functions.hpp"
#include "pros/misc.h"
#include "pros/motors.h"

int checkColour() {
	if (colourSortingSensor.get_hue() > 160 && colourSortingSensor.get_hue() < 320 && colourSortingSensor.get_proximity() > 160) {
		return 2; //blue
	}
	else if (colourSortingSensor.get_hue() < 10 && colourSortingSensor.get_proximity() > 160){
		return 1; //red
	} else {
        return 0; //background
    }
}

void nextState() {
    currentState += 1;
    if (currentState == 3 || currentState == 4 || currentState == 5 || currentState == 6) {
        currentState = 0;
    }
    if (currentState == 7) {
        currentState = 2;
    }
    target = state[currentState];
    if(currentState == 2) {
        firstTimeSwitch = true;
        intakeTarget = intake.get_position() - 200;
    }
}

void updateHookForLB() {
    if(lastCurrentState != currentState && lastCurrentState == 1) {
       firstTimeSwitch = true;
        intakeTarget = intake.get_position() - 200; 
    }
    lastCurrentState = currentState;
}

void liftControl() {
    double kp = 1.7;
    target = state[currentState];
    double error = target - LB_rotation.get_position()/100.0;
    double velocity = kp * error;
    if(!lbTip) {
        if(currentState == 2 && velocity > 25) {
            wallStake.move(127);
        } else {
            wallStake.move(velocity);
        }
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        firstTimeSwitch = false;
    }
    if(firstTimeSwitch) {
        if(intake.get_position() > intakeTarget) {
            intake.move(-60);
        } else {
            firstTimeSwitch = false;
        }
    }
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "%i", (int)LB_rotation.get_position()/100);
    // pros::screen::print(TEXT_LARGE_CENTER, 7, "%i", (int)velocity);
    //pros::screen::print(TEXT_LARGE_CENTER, 9, "%d", LB_rotation.get_position()/100);
}

void doubleLBLoad() {
    if(currentState == 0) {
        intakeJustStarted = true;
        currentState = 1;
    }
    if(!firstTimeSwitch) {
        intake.move(127);
    }
    if(currentState == 1 && intakeJamCheck()) {
        currentState = 6;
    }
}

void intakeControl() {
    intake.move(intakeSpeed);

    if(intakeSpeed != lastIntakeSpeed) {
        intakeJustStarted = true;
    }
    lastIntakeSpeed = intakeSpeed;

    if(intakeJamCheck() && intakeSpeed == 127 && currentState != 1) {
        intake.move(-100);
        pros::delay(150);
    }

    // if(currentState == 1 && intakeJamCheck()) {
    //     if(intake.get_actual_velocity() < 5) {
    //         intake.move(0);
    //     }
    // }
}

bool intakeJamCheck() {
    if(intake.get_actual_velocity() > 110) {
        intakeJustStarted = false;
    }

    if (!intakeJustStarted) {
        if(fabs(intake.get_actual_velocity()) < 5) {
            intakeJamCounter++;
            if(doubleLBLoading) {
                if(intakeJamCounter > 2) {
                    intakeJamCounter = 0;
                    return true;
                }
            } else {
               if(intakeJamCounter > 25) {
                    intakeJamCounter = 0;
                    return true;
                } 
            }
            return false;
        } else {
            intakeJamCounter = 0;
            return false;
        }
    } else {
        return false;
    }
}

void colourSort() {
    int ringState = checkColour();

    if (colourSortingSelector == 1 && ringState == 1) { //We are blue, ejecting red
        colourSortingEjectionState = true;
    } 
    else if (colourSortingSelector == 2 && ringState == 2) { //We are red, ejecting blue
        colourSortingEjectionState = true;
    }
    if (colourSortingEjectionState) {
        ejectCounter++;
        intake.move(-35);
        if (ejectCounter > 6) {
            colourSortingEjectionState = false;
            ejectCounter = 0;
        }
    }
}