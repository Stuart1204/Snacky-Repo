#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"

void initialize() {
	colourSortSensor.set_integration_time(10);
}
void disabled() {}

void competition_initialize() {}

void autonomous() {
	
}

void opcontrol() {
	leftMotors.set_brake_mode(MotorConfigs::BRAKE);
	rightMotors.set_brake_mode(MotorConfigs::BRAKE);
    while (true) {
		pros::delay(20);
		drivetrainControls();
		subsystemControls();
    }
}