#include "auton_functions.cpp"
#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
#include <sys/types.h>

void autonSkillsInDriver() {
	skillsAuton();
	pros::delay(10000);
}

void initialize() {
	chassis.calibrate();
	// parkSensor.set_led_pwm(100);
	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			pros::delay(20);
		}
	});

	allianceColour = AllianceColour::RED;
	startingSide = Selection::LONG_AND_CENTER;
	autonSelectionEnum = Selection::LONG_AND_CENTER;

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
	// skillsAuton();
	// runSelectedAuton();
	SAWP4Goal();
} 

void opcontrol() {
	odomLift.setState(true);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
    }
}