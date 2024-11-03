#include "teleOp_functions.hpp"
#include "pros/misc.h"

void mogoMechControls() {
    if(controller.get_digital_new_press(DIGITAL_R2)) {
			mogoState = !mogoState;
			if(mogoState) {
				mogoMech.set_value(true);
			} else {
				mogoMech.set_value(false);
			}
		}
}

void doinkerControls() {
    if(controller.get_digital_new_press(DIGITAL_A)) {
			doinkerState = !doinkerState;
			if(doinkerState) {
				doinker.set_value(true);
			} else {
				doinker.set_value(false);
			}
		}
}

void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX, false, 0.5);
}

void intakeControls() {
	if(controller.get_digital(DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move(60);
	} else if (controller.get_digital(DIGITAL_L1)) {
		if(colourSortingSelector == 0) {
			intake.move(127);
		} 
		else if (colourSortingSelector == 1) { //red
			if(ringColour == false && distanceSort.get() < 20) {
				colourSortingEjectionState = true;
			} else {
				intake.move(127);
			}
		} 
		else if (colourSortingSelector == 2) { //blue
			if(ringColour == true && distanceSort.get() < 20) {
				colourSortingEjectionState = true;
			} else {
				intake.move(127);
			}
		}
		
		if(colourSortingEjectionState) {
			// if(remainder(bIntake.get_position(), 360.0) > 180) {
			// 	counter = 0;
			// 	intake.move(0);
			// 	controller.rumble("-");
			// }
			// if(counter > 8) {
			// 	colourSortingEjectionState = false;
			// }
			intake.move(0);
			counterEject++;
			if (counterEject > 8) {
				colourSortingEjectionState = false;
				counterEject = 0;
			}
		}
	} 
	else if (controller.get_digital(DIGITAL_L2)) {
		intake.move(-127);
	} 
	else if (wallStakeLoad || climbState){
	} else {
		intake.move(0);
	}
}

void wallStakeControls() {
	if (controller.get_digital_new_press(DIGITAL_X)) {
		wallStakeLoad = !wallStakeLoad;
	} 
	if(wallStakeLoad) {
		wallStakeLoad = wallStakeLoading();
	}

	if (controller.get_digital_new_press(DIGITAL_R1)) {
		wallStakeScore = true;
		wallStakeDirection = true;
		climbState = false;
	}
	if(wallStakeScore) {
		wallStakeScore = wallStakeScoring();
	}
}

void climbControls() {
    if (controller.get_digital_new_press(DIGITAL_B)) {
		climbState = !climbState;
	}
	if (climbState) {
			if (wallStake.get_position() < 1500) {
				wallStake.move(80);
			} 
			else {
				intake.move(-80);
				wallStake.move(0);
			}
		} 
		else {
			if(wallStakeScore || wallStakeLoad || colourSortingEjectionState) {

			} else {
				wallStake.move(0);
			}
		}
	}

void checkColour() {
	if (colourSortingSensor.get_hue() > 80 && colourSortingSensor.get_hue() < 300) {
		ringColour = true; //blue
	}
	else if (colourSortingSensor.get_hue() < 20 || colourSortingSensor.get_hue() > 320){
		ringColour = false; //red
	}
}
