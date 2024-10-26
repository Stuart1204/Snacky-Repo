#include "main.h"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "globals.hpp"
#include "teleOp_functions.cpp"
#include "autons.cpp"

// pros screen formatting

void screen()
{ 
	// loop forever
	while (true)
	{
		lemlib::Pose pose = chassis.getPose();			// get the current position of the robot
		pros::lcd::print(0, "x: %f", pose.x);			// print the x position
		pros::lcd::print(1, "y: %f", pose.y);			// print the y position
		pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
		pros::delay(10);
	}
}

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed){	
		pros::lcd::set_text(2, "I was pressed!");
	}
	else {
		pros::lcd::clear_line(2);
	}
}

// initializing all robot functions
void initialize()
{

	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	wallStake.tare_position();
	wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	colourSortingSensor.disable_gesture();
	colourSortingSensor.set_led_pwm(80);
	bIntake.tare_position();

	// intializing the colour sorting. if colourSortingSelector = 0, No colour sorting
	// If 1, colour sorting Red (ejecting red)
	// If 2, colour sorting Blue (ejecting blue)
	colourSortingSelector = 0;
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	lemlib::Pose pose = chassis.getPose();
	printf("X: %f, Y: %f, Theta: %f\n ", pose.x, pose.y, pose.theta);
	
	nonMogoRushBlue();
}

void opcontrol()
{
	leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    while (true) {
		pros::delay(20);
		mogoMechControls();
		doinkerControls();
		drivetrainControls();
		intakeControls();
		wallStakeControls();
		climbControls();

		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
		pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
		pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
		pros::screen::print(TEXT_LARGE_CENTER, 10, "line sensor: %d", lineTracker.get_value());
    }
}