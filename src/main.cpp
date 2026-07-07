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

	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			//elevator.run();
			pros::delay(50);
		}
	});
	elevator.elevatorReset();

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
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
    }
}