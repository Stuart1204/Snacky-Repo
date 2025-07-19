#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"

void initialize() {
	chassis.calibrate();
	parkSensor.set_led_pwm(100);
	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			pros::delay(20);
		}
	});

	// allianceColour = AllianceColour::BLUE;
	// startingSide = Selection::RIGHT;
	// autonSelectionEnum = Selection::SOLO_AWP;
	autonSelection();
	// pros::delay(5000);
}
void disabled() {}

void competition_initialize() {}
void autonomous() {
	runSelectedAuton();
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