#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"

void initialize() {
	allianceColour = AllianceColour::RED;

	chassis.calibrate();
	parkSensor.set_led_pwm(100);
	pros::Task intakeTask([]{
		while (true) {
			intake.run();
			pros::delay(20);
		}
	});
}
void disabled() {}

void competition_initialize() {}

void autonomous() {
	skillsAuton();
}

void opcontrol() {
	leftMotors.set_brake_mode(MotorConfigs::BRAKE);
	rightMotors.set_brake_mode(MotorConfigs::BRAKE);
	odomLift.setState(true);
	pros::delay(3000);
	chassis.setPose(0,0, 180);
	resetPos();
	leftResetPos();
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
    }
}