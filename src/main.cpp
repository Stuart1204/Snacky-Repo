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

	allianceColour = 0; // nothing
	autonSelection();
}
void disabled() {}

void competition_initialize() {}

void autonomous() {
	rightLongGoalAndCenterAuton();
}

void opcontrol() {
	leftMotors.set_brake_mode(MotorConfigs::BRAKE);
	rightMotors.set_brake_mode(MotorConfigs::BRAKE);
	odomLift.setState(true);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
    }
}