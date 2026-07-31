#include "auton_functions.cpp"
#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "elevator.cpp"
#include "controls.cpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <sys/types.h>

#include "vision_drive.hpp"
VisionDriveController *vdc = nullptr;

void initialize()
{
	chassis.calibrate();
	static VisionDriveController controller(
		chassis,
		ai_vision,
		inertial_sensor,
		intakeMotor,
		5.0,  // camera 5 inches forward of tracking center
		14.0, // camera 8 inches above ground
		-30.0 // camera pitched 30° downward
	);
	controller.setupSensor();
	vdc = &controller;

	elevator.elevatorReset();
	pros::Task elevatorTask([]
							{
		while (true) {
			elevator.run();
			intake.run();
			pros::delay(40);
		} });

	allianceColour = AllianceColour::RED;
	startingSide = Selection::MATCH;
	autonSelectionEnum = Selection::MATCH;

	// autonSelection();

	if (autonSelectionEnum == Selection::SKILLS || startingSide == Selection::SKILLS)
	{
		enableColourSort = false;
	}
	else
	{
		enableColourSort = true;
	}
	// clamp.setState(true);
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
	// runSelectedAuton();
	TAWP();
}

void opcontrol()
{
	leftMotors.set_current_limit_all(2250);
	rightMotors.set_current_limit_all(2250);
	controller.print(0, 0, "%s", isNeutralGoal ? "NEUTRAL " : "ALLIANCE");
	// leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	while (true)
	{
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		pros::delay(20);
	}
}
