#include "teleOp_functions.hpp"
#include "lemlib/timer.hpp"

void mogoMechControls() {
    if (controller.get_digital_new_press(DIGITAL_R2)) {
		mogoState = !mogoState;
		if (mogoState) {
			mogoMech.set_value(true);
		} 
		else {
			mogoMech.set_value(false);
		}
	}
}

void diggerMechControls() {
    if (controller.get_digital_new_press(DIGITAL_Y)) {
		diggerState = !diggerState;
		if(diggerState) {
			diggerMech.set_value(true);
		} 
		else {
			diggerMech.set_value(false);
		}
	}
}

void wallStakeControls() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
		scoreStake = true;
	}

	if (scoreStake) {
		scoreStake = scoreWallStake();
	}
}

void drivetrainControls() {
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    chassis.arcade(leftY, rightX, 3.7);
}

// void initiateWallStakeLoading() {
//     if (controller.get_digital_new_press(DIGITAL_X)) {
//         loadStake = true;
// 		holdPosition = false;
// 		positionSetting = true;
// 	}

//     if (loadStake == true) {
// 		loadStake = loadWallStake();
// 	}
// }

void initiateWallStakeLoading() {
    if (controller.get_digital_new_press(DIGITAL_X)) {
		Xstate = !Xstate;
		if (Xstate) {
			loadStake = true;
			holdPosition = false;
			positionSetting = true;	
		} else {
			loadStake = false;
		}
	}
	if (loadStake == true) {
			loadStake = loadWallStake();
		if(loadStake == false) {
			Xstate = false;
		}
	}
}



void intakeControls() {
    if (controller.get_digital(DIGITAL_L1)) {   
		hook.move(127);
		intake.move(127);
		std::cout <<"currentT" <<pros::c::millis() <<" colour" <<ringTracker.get_hue() << " distance" <<ringTracker.get_proximity() <<" speed" <<hook.get_voltage() <<" counter" << counter<<"\n";
		if (ringTracker.get_hue() >= 120 && ringTracker.get_proximity()>= 130) { 
			counter = 0;
		}
		counter++;
		if (counter >= 1 && counter <= 4) {
			hook.move(0);
		}
	}
	else if (controller.get_digital(DIGITAL_L2)){
		hook.move(-127);
		intake.move(-127);
	}
	else if (loadStake == true){

	} 
	else {
		hook.move(0);
		intake.move(0);
	}
}

void wallStakeTipPositionControl() {
	if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		wallStakePositionState = !wallStakePositionState;
		wallStakeGoDown = true;
	}

	if(wallStakePositionState) {
		wallStakeTipMogoPosition();
	} 
	else {
		if(scoreStake == false) {
			if(wallStakeGoDown) {
				if(wallStake.get_position() > 20) {
					wallStake.move(-127);
				} 
				else {
					wallStake.move(0);
					wallStake.set_brake_mode(MOTOR_BRAKE_COAST);
					wallStakeGoDown = false;
				}
			}
		}
	}
}

void homing() {
    if (controller.get_digital(DIGITAL_A)) {
		int currentT = pros::c::millis();
		std::cout <<"currentT" <<currentT <<"\n";
		if (currentT - prevT >= interval) {
			prevT = currentT;
			std::cout <<"prevT"<<prevT << "\n";
			int currentD = wallStakeAligner.get();
			std::cout <<"currentD" <<currentD <<"\n";
			if (prevD != 0 && (prevD - currentD) >= 10 && prevD <= 150) {
				controller.rumble(". ");
				std::cout <<"prevD" <<prevD <<"\n";
			}
			prevD = currentD;
		} 
	}
}
 