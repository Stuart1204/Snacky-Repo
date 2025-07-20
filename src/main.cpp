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

	enableColourSort = false;

	allianceColour = AllianceColour::RED;
	startingSide = Selection::LEFT;
	autonSelectionEnum = Selection::SKILLS;
	//autonSelection();
}
void disabled() {}

void competition_initialize() {}
void autonomous() {
	// chassis.setPose(0,0,0);
	// turnToHeading(5);
	// turnToHeading(-83);
	// resetPos();
	// leftResetPos();
	// lemlib::Pose pose = chassis.getPose();
    // pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "y: %f", pose.y);
    // pros::screen::print(TEXT_LARGE_CENTER, 5, "heading: %f", pose.theta);
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