#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "controls.cpp"
#include "lemlib/pose.hpp"

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
	startingSide = Selection::LEFT;
	autonSelectionEnum = Selection::SKILLS;
	//autonSelection();
	if(autonSelectionEnum == Selection::SKILLS) {
		enableColourSort = false;
	} else {
		enableColourSort = true;
	}
}
void disabled() {}

void competition_initialize() {}
void autonomous() {
	//runSelectedAuton();
	chassis.setPose(0,0,-90);
	rightResetPos();
	resetPos();
	moveToPoint(-37, 48, false, true);
	turnToHeading(-90);
	driveToGoal(false);
	lemlib::Pose pose = chassis.getPose();
	pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
	pros::screen::print(TEXT_LARGE_CENTER, 3, "y: %f", pose.y);
	pros::screen::print(TEXT_LARGE_CENTER, 5, "heading: %f", pose.theta);
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