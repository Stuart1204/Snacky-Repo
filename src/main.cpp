#include "globals.hpp"
#include "autons.cpp"
#include "claw.cpp"
#include "intakevator.cpp"
#include "turntable.cpp"
#include "controls.cpp"
#include "pros/rtos.hpp"
#include "pin_colour.hpp"

void initialize()
{
	chassis.calibrate();
	// Set the chassis/raw telemetry pose before MCL begins field-wide initialization.
	localization.resetPose(-48, -48, 0.0f);
	localization.start();
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
	// Clear the autonomous selector before drawing the opcontrol telemetry page.
	pros::screen::erase();

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