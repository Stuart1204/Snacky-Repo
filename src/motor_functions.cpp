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
	if (colourSortingSensor.get_hue() > 80 && colourSortingSensor.get_hue() < 300 && colourSortingSensor.get_proximity() > 160) {
		ringColour = true; //blue
	}
	else if (colourSortingSensor.get_hue() < 20 || colourSortingSensor.get_hue() > 320 && colourSortingSensor.get_proximity() > 160){
		ringColour = false; //red
	}
}

void nextState() {
    currentState += 1;
    if (currentState == 3 || currentState == 4) {
        currentState = 0;
    } 
    target = state[currentState];
    if(currentState == 2) {
        firstTimeSwitch = true;
        intakeTarget = secondStageIntake.get_position() - 100;
    }
}

void liftControl() {
    double kp = 1.2;
    target = state[currentState];
    double error = target - LB_rotation.get_position()/100;
    double velocity = kp * error;
    if(!lbTip) {
        wallStake.move(velocity);
    }
    if(firstTimeSwitch) {
        if(secondStageIntake.get_position() > intakeTarget) {
            secondStageIntake.move(-20);
        } else {
            firstTimeSwitch = false;
        }
    }
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "%i", (int)LB_rotation.get_position()/100);
    // pros::screen::print(TEXT_LARGE_CENTER, 7, "%i", (int)velocity);
    //pros::screen::print(TEXT_LARGE_CENTER, 9, "%d", LB_rotation.get_position()/100);
}
