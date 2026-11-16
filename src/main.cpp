#include "globals.hpp"
#include "autons.cpp"
#include "claw.cpp"
#include "intakevator.cpp"
#include "pros/abstract_motor.hpp"
#include "turntable.cpp"
#include "controls.cpp"
#include "pros/rtos.hpp"
#include "pin_colour.hpp"

void initialize()
{
	chassis.calibrate();
	// Set the chassis/raw telemetry pose before MCL begins field-wide initialization.
	// localization.resetPose(-48, -48, 0.0f);
	// localization.start();
	pinColourDetector.initialize();

	intakevator.resetEncoders();
	// Runs the claw, intakevator, and turntable position controller.
	pros::Task intakevatorTask([]
							{
		while (true) {
			claw.run();
			intakevator.run();
			turntable.run();
			pros::delay(40);
		} });
	// Optical sampling is read-only; it never commands the intake or elevator.
	pros::Task pinColourTask([]
							{
		while (true) {
			pinColourDetector.update();
			pros::delay(20);
		} });

	allianceColour = AllianceColour::RED;
	startingSide = Selection::LEFT;
	autonSelectionEnum = Selection::ELIMS;

	// autonSelection();
	// clampSelection();
	clamp.setState(true);

}

void disabled() {
	clamp.setState(false);
}

void competition_initialize() {}

void autonomous()
{
	isAutonomous = true;
}


void opcontrol()
{
	isAutoReset = true;
	isAutonomous = false;
	controller.print(0, 0, "%s", isNeutralGoal ? "NEUTRAL " : "ALLIANCE");
	// Clear the autonomous selector before drawing the opcontrol telemetry page.
	pros::screen::erase();
	leftMotors.set_brake_mode(pros::MotorBrake::brake);
	rightMotors.set_brake_mode(pros::MotorBrake::brake);

	while (true)
	{
		subsystemControls();
		drivetrainControls();
		displayData();
		pros::delay(20);
	}
}