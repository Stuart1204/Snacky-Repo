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

	autonSelection();

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
	chassis.setPose(28, 48, 90);
    // --------------- GOING TO CLEAR PARK ZONE
    turnToHeading(150, true);
    moveToPoint(41.5, 18, true, false, true);
    turnToHeading(-180, false);
    leftResetPos();
    resetPos();
    moveToPoint(41.5, 0, true, false, false);
    turnToHeading(90, true);
	frontClearParkZone();
	moveToPoint(19, 19, true, true, false);
	moveToPoint(15, 15, true, true, true);
	intakeLift.setState(true);
	intake.setState(IntakeState::SCORELOW);
	driveToGoal(true);
	intakeShake(1200);
    pros::delay(2000);
} 

void opcontrol() {
	leftDoinker.setState(false);
	rightDoinker.setState(false);
	odomLift.setState(true);
    while (true) {
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
    }
}