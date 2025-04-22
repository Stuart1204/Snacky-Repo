#include "motor_functions.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


// bool wallStakeLoading() {
//     if(intake.get_actual_velocity() < 3 && counter > 20) {
//         inWallStake = true;
//     } else {
//         counter++;
//     }

//     if(inWallStake) { 
//         intake.move(0);
//         inWallStake = false;
//         counter = 0;
//         return false;
//     }

//     if(!inWallStake) { 
//         intake.move(127);
//         if(wallStake.get_position() < 250 && !exitLoadWallStake) {
//             wallStake.move(30);
//         } else {
//             wallStake.move(2);
//         }
//         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
//             wallStake.move(12);
//         }
//         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
//             wallStake.move(-2);
//             exitLoadWallStake = true;
//         }
//         return true;
//     }
// }

// bool wallStakeScoring() {
//     if(wallStakeDirection) {
//         if(wallStake.get_position() < 1350) {
//             wallStake.move(127);
//             if(wallStake.get_position() < 700) {
//                 intake.move(-25);
//             } else {
//                 intake.move(0);
//             }
//         } else {
//             wallStakeDirection = false;
//         }
//         return true;
//     } else {
//         if(wallStake.get_position() > 40) {
//             wallStake.move(-127);
//             return true;
//         } else {
//             wallStake.move(0);
//             return false;
//         }
//     }

// }

void checkColour() {
	if (colourSortingSensor.get_hue() > 80 && colourSortingSensor.get_hue() < 300 && colourSortingSensor.get_proximity() > 220) {
		ringColour = 2; //blue
	}
	else if ((colourSortingSensor.get_hue() < 20 || colourSortingSensor.get_hue() > 32) && colourSortingSensor.get_proximity() > 220){
		ringColour = 1; //red
	}
    else {
        ringColour = 0; //background
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
        intakeTarget = intake.get_position() - 100;
    }
}

void liftControl() {
    double kp = 3;
    target = state[currentState];
    double error = target - LB_rotation.get_position()/100;
    double velocity = kp * error;
    if(!lbTip) {
        if(currentState == 2 && velocity > 10) {
            wallStake.move(127);
        } else {
            wallStake.move(velocity);
        }
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        firstTimeSwitch = false;
    }
    if(firstTimeSwitch) {
        if(intake.get_position() > intakeTarget) {
            intake.move(-30);
        } else {
            firstTimeSwitch = false;
        }
    }
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "%i", (int)LB_rotation.get_position()/100);
    // pros::screen::print(TEXT_LARGE_CENTER, 7, "%i", (int)velocity);
    //pros::screen::print(TEXT_LARGE_CENTER, 9, "%d", LB_rotation.get_position()/100);
}

void intakeControl() {
    intake.move(intakeSpeed);

    if(intakeSpeed != lastIntakeSpeed) {
        intakeJustStarted = true;
    }
    lastIntakeSpeed = intakeSpeed;

    if(intakeJamCheck() && !intakeSpeed == 0 && currentState != 1) {
        intake.move(-100);
        pros::delay(150);
    }

    if(currentState == 1 && intakeJamCheck()) {
        if(intake.get_actual_velocity() < 5) {
            intake.move(0);
        }
    }
}

bool intakeJamCheck() {
    if(intake.get_actual_velocity() > 100) {
        intakeJustStarted = false;
    }

    if (!intakeJustStarted) {
        if(fabs(intake.get_actual_velocity()) < 10) {
            intakeJamCounter++;
            if(intakeJamCounter > 3) {
                return true;
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
    if (colourSortingSelector == 1 && ringColour == 1) { //We are blue, ejecting red
        colourSortingEjectionState = true;
    } 
    else if (colourSortingSelector == 2 && ringColour == 2) { //We are red, ejecting blue
        colourSortingEjectionState = true;
    }
    if (colourSortingEjectionState) {
        ejectCounter++;
        intake.move(-50);
        if (ejectCounter > 15) {
            colourSortingEjectionState = false;
            ejectCounter = 0;
        }
    }
}