#include "motor_functions.hpp"
#include "pros/motors.h"

double destinationPosition;

bool loadWallStake() {
	wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	if (holdPosition && positionSetting) {
		destinationPosition = hook.get_position() - 1200;
		positionSetting = false;
	}
	
	if(!holdPosition) {
		intake.move(100);
		hook.move(75);
		if (ringTracker.get_proximity() > 180) {
			holdPosition = true;
		}
        return true;
	} 
	else if (hook.get_position() > destinationPosition) {
		intake.move(-70);
		hook.move(-70);
        return true;
	} 
	else {
		intake.move(0);
		hook.move(0);
		ringInWallStake = true;
		pros::screen::print(pros::E_TEXT_SMALL, 4, "bool: %d", ringInWallStake);
		return false;
	}
}

bool scoreWallStake() {
	wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	if (armDirection) {
		if(wallStake.get_position() < 1100) {
			wallStake.move(127);
		} 
		else {
			armDirection = false;
		}
		return true;
	} 
	else {
		if (wallStake.get_position() > 10) {
			wallStake.move(-127);
			return true;
		} 
		else {
			armDirection = true;
			wallStake.move(0);
			ringInWallStake = false;
			return false;
		}
	}
}

void wallStakeTipMogoPosition() {
	if (wallStake.get_position() < 1600) {
		wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
		wallStake.move(70);
	} 
	else {
		wallStake.set_brake_mode(MOTOR_BRAKE_BRAKE);

		if(wallStake.get_position() < 1600) {
			wallStake.move(5);
		} else {
			wallStake.move(-6);
		}
	}
}

void spinIntake(int speed) {
	hook.move(speed);
	intake.move(speed);
}

void reverseIntake() {
	hook.move(-127);
	intake.move(-127);
}

void stopIntake() {
	hook.move(0);
	intake.move(0);
}