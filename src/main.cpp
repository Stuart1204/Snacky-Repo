#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/rtos.hpp"

void initialize() {
	chassis.calibrate();
	parkSensor.set_led_pwm(100);
	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			if(!inertial_sensor.is_installed()) {
				printf("disconnected!");
			}
			pros::delay(20);
		}
	});

	allianceColour = AllianceColour::RED;
	startingSide = Selection::RIGHT;
	autonSelectionEnum = Selection::SKILLS;
	//autonSelection();
	if(autonSelectionEnum == Selection::SKILLS || startingSide == Selection::SKILLS) {
		enableColourSort = false;
	} else {
		enableColourSort = true;
	}
}
void disabled() {}

void competition_initialize() {}
void autonomous() {
	//skillsAuton();
	runSelectedAuton();
	//NISIGSAWP();
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