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

	elevator.elevatorReset();
	pros::Task elevatorTask([]
							{
		while (true) {
			elevator.run();
			intake.run();
			pros::delay(40);
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

	// runSelectedAuton();
	elimsAutoConsistentRight();
	// iceAutoRight();
	// skillsAuton();
	// TAWPRight();
	// iceAutoRight();
	// elimsAllianceGoalRight();
	// elimsAllianceGoal();
	// skillsAuton();
	// TAWPLeft();
	// TAWPRight();
	// chassis.setPose(0,0,250);
	// goalJiggle(270, 1000);
	// TAWPRight();
	// hybridRight();
}


void opcontrol()
{

	if (startingSide == Selection::SKILLS || autonSelectionEnum == Selection::SKILLS) {
		skillsTask = new pros::Task([]{
			skillsDriverMacro();
		});
		isSkillsMacro = true;
	}

	isAutoReset = true;
	isAutonomous = false;
	controller.print(0, 0, "%s", isNeutralGoal ? "NEUTRAL " : "ALLIANCE");

	while (true)
	{
		if(!isSkillsMacro) {
			drivetrainControls();
    	}
		subsystemControls();
		displayData();
		pros::delay(20);

		if (controller.get_digital_new_press(Buttons::DOWN) || 
			abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) > 20 || 
				abs(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) > 20) {
			if (skillsTask != nullptr) {
				skillsTask->suspend();
				isSkillsMacro = false;
			}
		}
	}
}