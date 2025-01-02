#include "main.h"
#include "lemlib/api.hpp"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "globals.hpp"
#include "teleOp_functions.cpp"
#include "autons.cpp"

// pros screen formatting
pros::screen_touch_status_s_t status;

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
	//wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	colourSortingSensor.disable_gesture();
	colourSortingSensor.set_led_pwm(80);
	// intializing the colour sorting. if colourSortingSelector = 0, No colour sorting
	// If 1, colour sorting Red (ejecting red)
	// If 2, colour sorting Blue (ejecting blue)
	colourSortingSelector = 0;
	pros::screen::set_eraser(0x00000000); 
	while (true){
		break;
		pros::delay(20);
		pros::screen::set_pen(0x00FF0000);
		pros::screen::fill_rect(0,0,150,100);
		pros::screen::fill_rect(0,150, 150, 240);
		pros::screen::set_pen(0x000000FF);
		pros::screen::fill_rect(200,0,350,100);
		pros::screen::fill_rect(200,150, 350, 240);
		pros::screen::set_pen(0x00008000);
		pros::screen::fill_rect(400,0,475,240);
		pros::screen::set_pen(0x00FFFFFF);
		pros::screen::print(pros::E_TEXT_MEDIUM, 20,35, "Red Positive"); 
		pros::screen::print(pros::E_TEXT_MEDIUM, 20,185, "Red Negative"); 
		pros::screen::print(pros::E_TEXT_MEDIUM, 215,35, "Blue Positive"); 
		pros::screen::print(pros::E_TEXT_MEDIUM, 215,185, "Blue Negative"); 
		pros::screen::print(pros::E_TEXT_MEDIUM, 405,115, "Skills");
		status = pros::screen::touch_status();
		if (status.x > 0 && status.x < 150 && status.y > 0 && status.y < 100) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 20,35, "Selected     "); 
			autonSelection = 1;
			break;
		}
		if (status.x > 0 && status.x < 150 && status.y > 150 && status.y < 240) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 20,185, "Selected     "); 
			autonSelection = 2;
			break;
		}
		if (status.x > 200 && status.x < 350 && status.y > 0 && status.y < 100) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 215,35, "Selected     "); 
			autonSelection = 3;
			break;
		}
		if (status.x > 200 && status.x < 350 && status.y > 150 && status.y < 240) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 215,185, "Selected     "); 
			autonSelection = 4;
			break;
		}
		if (status.x > 400 && status.x < 480 && status.y > 0  && status.y < 240) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 405,115, "Selected     "); 
			autonSelection = 5;
			break;
		}
	}
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	if (autonSelection == 1) {
		mogoRushRed();
	}
	else if (autonSelection == 2) {
		nonMogoRushRed();
	}
	else if (autonSelection == 3) {
		mogoRushBlue();
	}
	else if (autonSelection == 4) {
		nonMogoRushBlue();
	}
	else if (autonSelection == 5) {
		skillsAuton();
	}
}

void opcontrol()
{
	if (autonSelection == 1) {
		colourSortingSelector = 2;
	}
	else if (autonSelection == 2) {
		colourSortingSelector = 2;
	}
	else if (autonSelection == 3) {
		colourSortingSelector = 1;
	}
	else if (autonSelection == 4) {
		colourSortingSelector = 1;
	}
	else if (autonSelection == 5) {
		colourSortingSelector = 0;
	}
	multitaskIntakeControl.suspend();
	leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	wallStake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	pros::screen::erase();
    while (true) {
		pros::delay(20);
		mogoMechControls();
		drivetrainControls();
		intakeControls();
		wallStakeControls();
		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_LARGE_CENTER, 10, "dist sensor: %d", distanceSort.get());
		// pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
		// pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
		// pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    }
}