#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/pose.hpp"
#include "pros/colors.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "globals.hpp"
#include "teleOp_functions.cpp"
#include "autons.hpp"

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
	leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	wallStake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	wallStake.tare_position();
	ringTracker.disable_gesture();
	ringTracker.set_led_pwm(50);
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
	pros::lcd::register_btn1_cb(on_center_button);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	// set position to x:0, y:0, heading:0

	mogoRushBlue();

	lemlib::Pose pose = chassis.getPose();
	pros::screen::print(pros::E_TEXT_SMALL, 3, "X: %f, Y: %f, Theta: %f\n ", pose.x, pose.y, pose.theta);
}

void opcontrol()
{
	lemlib::Pose pose = chassis.getPose();
    while (true) {
		pros::delay(20);
		mogoMechControls();
		diggerMechControls();
		wallStakeControls();
		drivetrainControls();
		initiateWallStakeLoading();
		intakeControls();
		homing();
		wallStakeTipPositionControl();
		// pros::screen::print(TEXT_MEDIUM, 3, "wallStake proximity%i\n", wallStakeAligner.get_proximity());
		pose = chassis.getPose();			// get the current position of the robot
		pros::screen::print(TEXT_MEDIUM,0, "x: %f", pose.x);			// print the x position
		pros::screen::print(TEXT_MEDIUM,1, "y: %f", pose.y);			// print the y position
		pros::screen::print(TEXT_MEDIUM,2, "heading: %f", pose.theta); // print the heading
		std::cout <<"distance" << wallStakeAligner.get(); "\n";
    }
} 