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

void mogoStealControls() {
    if(controller.get_digital_new_press(DIGITAL_B)) {
			stealState = !stealState;
			if(stealState) {
				mogoMech.set_value(true);
				mogoSteal.set_value(true);
			} else {
				mogoMech.set_value(false);
				mogoSteal.set_value(false);
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

void descoreControls() {
	if(controller.get_digital_new_press(DIGITAL_Y)) {
			descoreState = !descoreState;
			descoreCounter = 0;
	}
	if(descoreState == true) {
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
			descoreState = false;
			descoreCounter = 0;
		}
	}
	if(descoreState) {
		descoreCounter++;
		if(wallStake.get_position() < 1000) {
			wallStake.move(127);
		} 
		else {
			wallStake.move(0);
		}

		if(descoreCounter > 5) {
			descoreLifter.set_value(HIGH);
		}
		if(descoreCounter > 28) {
			descorePuncher.set_value(HIGH);
		}
	} 
	else {
		descoreLifter.set_value(false);
		descorePuncher.set_value(false);
		descoreCounter ++;
		if (descoreCounter < 30) {
			wallStake.move(0);
		} 
	}
}

void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX * 0.8, false, 0.5);
}

void intakeControls() {
	if(controller.get_digital(DIGITAL_L1) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
		intake.move(60);
	} else if (controller.get_digital(DIGITAL_L1)) {
		// if(colourSortingSelector == 0) {
		// 	intake.move(127);
		// } 
		// else if (colourSortingSelector == 1) { //red
		// 	if(ringColour == false && distanceSort.get() < 200) {
		// 		colourSortingEjectionState = true;
		// 	} else {
		// 		intake.move(127);
		// 	}
		// } 
		// else if (colourSortingSelector == 2) { //blue
		// 	if(ringColour == true && distanceSort.get() < 200) {
		// 		colourSortingEjectionState = true;
		// 	} else {
		// 		intake.move(127);
		// 	}
		// }
		
		// if(colourSortingEjectionState) {
		// 	// if(remainder(bIntake.get_position(), 360.0) > 180) {
		// 	// 	counter = 0;
		// 	// 	intake.move(0);
		// 	// 	controller.rumble("-");
		// 	// }
		// 	// if(counter > 8) {
		// 	// 	colourSortingEjectionState = false;
		// 	// }
		// 	intake.move(-30);
		// 	counterEject++;
		// 	if (counterEject > 8) {
		// 		colourSortingEjectionState = false;
		// 		counterEject = 0;
		// 	}
		// }
		intake.move(127);
	} 
	else if (controller.get_digital(DIGITAL_L2)) {
		intake.move(-127);
	} 
	else if (wallStakeLoad || climbState){
	} else {
		intake.move(0);
	}
}

// void wallStakeControls() {
// 	if (controller.get_digital_new_press(DIGITAL_X)) {
// 		wallStakeLoad = !wallStakeLoad;
// 		exitLoadWallStake=false; 
// 	} 
// 	if(wallStakeLoad) {
// 		wallStakeLoad = wallStakeLoading();
// 	}

// 	if (controller.get_digital_new_press(DIGITAL_R1)) {
// 		if(!descoreState) {
// 			wallStakeScore = true;
// 			wallStakeDirection = true;
// 			climbState = false;
// 		}
// 	}
// 	if(wallStakeScore) {
// 		wallStakeScore = wallStakeScoring();
// 	}
// }

void climbControls() {
    if (controller.get_digital_new_press(DIGITAL_B)) {
		climbState = !climbState;
	}
	if (climbState) {
		if (wallStake.get_position() < 1440) {
			wallStake.move(80);
		} 
		else {
			intake.move(-80);
			wallStake.move(0);
		}
	} 
	else {
		if(wallStakeScore || wallStakeLoad || colourSortingEjectionState || descoreState) {
		
		} 
		else {
			if(descoreCounter > 30) { 
				if(wallStake.get_position() > 300) {
					wallStake.move(-127);
				} else {
					wallStake.move(0);
				}
			}
		}
	}
}
void colourSortButton() {
	if (controller.get_digital_new_press(DIGITAL_UP)) {
		colourSortButtonState = !colourSortButtonState;
		controller.rumble("-");
	}
	if (colourSortButtonState) {
		if(autonSelection == 1 || autonSelection == 2 || autonSelection == 5) {
			colourSortingSelector = 2;
		}
		else if (autonSelection == 3 || autonSelection == 4){
			colourSortingSelector = 1;
		}
		controller.print(0, 0, "ENABLED");
	}
	else {
		colourSortingSelector = 0;
		controller.print(0, 0, "DISABLED");
	}
}

void newWallStake(){
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
		nextState();

	}
}

void allianceStakeControls(){
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
		currentState = 3;
	}
}