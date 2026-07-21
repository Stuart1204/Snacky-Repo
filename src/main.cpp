#include "auton_functions.cpp"
#include "globals.hpp"
#include "autons.cpp"
#include "intake.cpp"
#include "elevator.cpp"
#include "controls.cpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/misc.hpp"
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
		13.0, // camera 8 inches above ground
		-25.0 // camera pitched 30° downward
	);
	controller.setupSensor();
	vdc = &controller;

	elevator.elevatorReset();
	pros::Task elevatorTask([]
							{
		while (true) {
			elevator.run();
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
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
	runSelectedAuton();
	chassis.setPose(0, 0, 0);
	intakeLift.setState(true);
	vdc->driveToObject(0.0, 24.0, GameObjectType::BLUE_YELLOW_PIN, true);
}

void opcontrol()
{
	leftMotors.set_current_limit_all(2250);
	rightMotors.set_current_limit_all(2250);
	// leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	// rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	while (true)
	{
		// intake has its own thread
		drivetrainControls();
		subsystemControls();
		displayData();
		intake.run();
		pros::delay(20);
	}
}
