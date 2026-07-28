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
	clamp.setState(true);
	allianceColour=2;
	chassis.setPose(60, 13, 108);
	//chassis.setPose(0,0,0);
	// Move elevator to 5, wrist holds current
	elevator.moveTo(7, -90);
	//elevator.waitUntilComplete();

	rightMotors.move(127);
	leftMotors.move(60);
	pros::delay(500);
	leftMotors.move(0);
	rightMotors.move(0);
	//chassis.turnToPoint(48, 24, 1000, {.forwards = false});
	chassis.moveToPoint(49, 26, 1500, {.forwards = false, .minSpeed = 60});
	lemlib::Pose pose = chassis.getPose();
    printf("X: %f", pose.x);
    printf("Y: %f", pose.y);
	leftResetPos();
	// //goalReset();
	elevator.setState(ElevatorState::SCORE);
	elevator.waitUntilPhase(2, 750);
	clamp.setState(false);
	pros::delay(600);
	elevator.setState(ElevatorState::RESET);
	//elevator.waitUntilComplete();
	chassis.moveToPoint(59, 0, 1000, {.earlyExitRange = 3});
	chassis.turnToHeading(270, 1000, {.earlyExitRange = 3});
	vdc->acquireAlliancePin(24,0, 15, true, 4000, false);
	chassis.turnToPoint(48, -24,1000, {.forwards = false});
	chassis.moveToPoint(46, -20, 1000, {.forwards = false, .maxSpeed = 70});
	// pose = chassis.getPose();
	// // printf("X: %f", pose.x);
    // // printf("Y: %f", pose.y);
	// // goalReset();
	chassis.waitUntilDone();
	elevator.waitUntilComplete();
	elevator.setState(ElevatorState::SCORE);
	pros::delay(600);
	chassis.moveToPoint(50, 0, 1000, {.earlyExitRange = 3});
	elevator.setState(ElevatorState::RESET);
	chassis.turnToPoint(24, 24, 1000, {.earlyExitRange = 3});
	vdc->acquireAlliancePin(23, 24, 15, true, 3000, true);
	chassis.turnToHeading(270, 1000);
	chassis.moveToPoint(45, 24, 1000, {.forwards = false, .earlyExitRange = 2});
	pose = chassis.getPose();
	printf("X: %f", pose.x);
    printf("Y: %f", pose.y);
	goalReset();
	rightResetPos();
	elevator.setState(ElevatorState::SCORE);
	elevator.waitUntilPhase(2, 750);
	pros::delay(600);
	// leftMotors.move(100);
	// rightMotors.move(-50);
	// pros::delay(200);
	// leftMotors.move(0);
	// rightMotors.move(0);
	// chassis.turnToPoint(48, 48, 1000);
	// vdc->acquireAlliancePin(48, 48, 24, true, 3000, true);
	// chassis.turnToHeading(90, 1000);
	// chassis.moveToPoint(24, 48, 1000, {.forwards = false});
	// pose = chassis.getPose();
	// printf("X: %f", pose.x);
    // printf("Y: %f", pose.y);
	// goalReset();
	// leftResetPos();
	// elevator.setState(ElevatorState::SCORE);
	// elevator.waitUntilPhase(2, 750);
	// pros::delay(600);
	// elevator.setState(ElevatorState::RESET);
	// rightMotors.move(100);
	// leftMotors.move(-50);
	// pros::delay(200);
	// leftMotors.move(0);
	// rightMotors.move(0);
	// vdc->acquireAlliancePin(0, 24, 15, true, 3000, false);
	// chassis.turnToPoint(-24, 48, 1000, {.forwards = false});
	// elevator.setState(ElevatorState::SCORE);
	// elevator.waitUntilPhase(2, 750);
	// pros::delay(600);



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
