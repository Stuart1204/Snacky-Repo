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
			elevator.run();
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
	// leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		// if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
		// 	leftElevator.move(127);
		// 	rightElevator.move(-127);
		// } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		// 	leftElevator.move(-127);
		// 	rightElevator.move(127);
		// } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
		// 	leftElevator.move(-127);
		// 	rightElevator.move(-127);
		// } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
		// 	leftElevator.move(127);
		// 	rightElevator.move(127);
		// } else {
		// 	leftElevator.move(0);
		// 	rightElevator.move(0);
		// }
		pros::delay(20);
    }
}