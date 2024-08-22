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
		pros::screen::print(pros::E_TEXT_SMALL, 1, "difference: %f", hook.get_position() - destinationPosition);
        return true;
	} 
	else {
		intake.move(0);
		hook.move(0);
		return false;
	}
}

bool scoreWallStake() {
	wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	if (armDirection) {
		if(wallStake.get_position() < 1000) {
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
			return false;
		}
	}
}

void wallStakeTipMogoPosition() {
	if (wallStake.get_position() < 1500) {
		wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
		wallStake.move(70);
	} 
	else {
		wallStake.move(0);
		wallStake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	}
}
