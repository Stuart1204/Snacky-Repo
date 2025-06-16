#include "main.h"
#include "lemlib/api.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "globals.hpp"
#include "teleOp_functions.cpp"
#include "autons.cpp"
#include "intake.cpp"



// pros screen formatting
pros::screen_touch_status_s_t status;

void screen()
{ 
	// loop forever
	while (true)
	{
		lemlib::Pose pose = chassis.getPose();			// get the current position of the robot
		// pros::lcd::print(0, "x: %f", pose.x);			// print the x position
		// pros::lcd::print(1, "y: %f", pose.y);			// print the y position
		// pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
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
void initialize() {

}
void disabled() {}

void competition_initialize() {}

void autonomous() {
	
}
void opcontrol()
{
    while (true) {
		pros::delay(20);
		
		
    }
}