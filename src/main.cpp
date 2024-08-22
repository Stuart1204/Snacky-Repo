#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "globals.hpp"
#include "teleOp_functions.cpp"

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

void runIntake() {
	hook.move(127);
	intake.move(127);
}
void stopIntake() {
	hook.move(0);
	intake.move(0);
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
    chassis.setPose(-165, 45, 180);
	chassis.moveToPose(-165, 0, 90, 2000);
	runIntake();
	lemlib::Timer timer1(750);
	stopIntake();
	chassis.moveToPose(-120, 0, 90, 2000);
	stopIntake();runIntake();
	lemlib::Timer timer2(100);
	stopIntake();
	chassis.moveToPose(-70, 60, 270, 2000);
	mogoMech.set_value(true);
	chassis.moveToPose(-60, 120, 0, 2000);
	runIntake();
	lemlib::Timer timer3(750);
	stopIntake();
	chassis.moveToPose(-60,0, 180 , 2000);
	chassis.waitUntilDone();
	lemlib::Pose pose = chassis.getPose();
	printf("X: %f, Y: %f, Theta: %f\n ", pose.x, pose.y, pose.theta);
}

void opcontrol()
{
	// int counter = 20;
    while (true) {
		pros::delay(20);
		mogoMechControls();
		diggerMechControls();
		wallStakeControls();
		drivetrainControls();
		initiateWallStakeLoading();
		intakeControls();
		wallStakeTipPositionControl();
		
		
		std::cout << pros::millis();
		std::cout << "hello" <<"\n";
		// pros::screen::print(pros::E_TEXT_SMALL, 3, "position: %f", wallStake.get_position());
		// std::cout <<"hello" <<"\n";
    }
}