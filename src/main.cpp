#include "auton_functions.cpp"
#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "elevator.cpp"
#include "controls.cpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <sys/types.h>

void initialize() {
	chassis.calibrate();

	elevator.elevatorReset();
	pros::Task elevatorTask([]{
		while (true) {
			elevator.run();
			pros::delay(40);
		}
	});

	allianceColour = AllianceColour::RED;
	startingSide = Selection::MATCH;
	autonSelectionEnum = Selection::MATCH;


	// autonSelection();

	if(autonSelectionEnum == Selection::SKILLS || startingSide == Selection::SKILLS) {
		enableColourSort = false;
	} else {
		enableColourSort = true;
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	runSelectedAuton();
} 

void opcontrol() {
	leftMotors.set_current_limit_all(2250);
	rightMotors.set_current_limit_all(2250);
	// leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		intake.run();
		pros::delay(20);
    }
}