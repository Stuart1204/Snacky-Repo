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

void initialize() {
	chassis.calibrate();
	parkSensor.set_led_pwm(100);
	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			pros::delay(20);
		}
	});

	allianceColour = AllianceColour::RED;
	startingSide = Selection::RIGHT;
	autonSelectionEnum = Selection::SKILLS;

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
	rightSevenBlock();
	// NISAWP();
	// runSelectedAuton();
} 

void opcontrol() {
	bool autoDoublePark = false; 
	u_int32_t teleOpStartTime;
	if(!pros::competition::is_autonomous() && !pros::competition::is_disabled() && pros::competition::is_connected()) {
		teleOpStartTime = pros::millis();
		autoDoublePark = true;
	}

	odomLift.setState(true);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);

		if(autoDoublePark) {
			if(pros::millis() - teleOpStartTime > 134900) {
				park.setState(true);
			}
		}
    }
}