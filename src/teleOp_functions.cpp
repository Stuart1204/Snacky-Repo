#include "teleOp_functions.hpp"

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

void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX, false, 0.5);
}

void intakeControls() {
    if (controller.get_digital(DIGITAL_L1)) {
		intake.move(127);
	} 
	else if (controller.get_digital(DIGITAL_L2)) {
		intake.move(-127);
	} 
	else {
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
				wallStake.move(0);
			}
		} 
		else {
			if(wallStakeScore || wallStakeLoad) {

			} else {
				wallStake.move(0);
			}
		}
	}
