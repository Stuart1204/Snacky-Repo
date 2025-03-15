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
void initialize()
{
	chassis.calibrate();
	chassis.setPose(0, 0, 0);
	wallStake.tare_position();
	rightMotors.set_zero_position_all(0);
	// rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	// leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
	leftMotors.set_zero_position_all(0);
	// colourSortingSelector = 2;
	lBDriveMotor.tare_position_all();
	if(LB_rotation.get_position()/100 > 350 && LB_rotation.get_position()/100 < 360) {
		LB_rotation.reset_position();
	}
	//wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
	colourSortingSensor.disable_gesture();
	colourSortingSensor.set_led_pwm(50);
	// intializing the colour sorting. if colourSortingSelector = 0, No colour sorting
	// If 1, colour sorting Red (ejecting red)
	// If 2, colour sorting Blue (ejecting blue)
	pros::screen::set_eraser(0x00000000); 
	while (true){
		//break; // remove 
		pros::delay(30);
		pros::screen::set_pen(0x00FFD700);
		pros::screen::fill_rect(0,70,130,170);
		pros::screen::set_pen(0x00FF69B4);
		pros::screen::fill_rect(200,70,340,170);
		pros::screen::set_pen(0x00008000);
		pros::screen::fill_rect(400,0,475,240);
		pros::screen::set_pen(0x00FFFFFF);
		pros::screen::print(pros::E_TEXT_MEDIUM, 45,110, "Quals"); 
		pros::screen::print(pros::E_TEXT_MEDIUM, 250,110, "Elims");  
		pros::screen::print(pros::E_TEXT_MEDIUM, 405,110, "Skills");
		status = pros::screen::touch_status();
		if (status.x > 0 && status.x < 130 && status.y > 70 && status.y < 170) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 30, 110, "Selected"); 
			pros::delay(500);
			pros::screen::set_eraser(0x00000000);
			pros::screen::erase();
			while (true) {
				pros::delay(30);
				pros::screen::set_pen(0x00FF0000);
				pros::screen::fill_rect(0,0,200,70);
				pros::screen::fill_rect(0,170, 200, 240);
				pros::screen::set_pen(0x000000FF);
				pros::screen::fill_rect(270,0,490,70);
				pros::screen::fill_rect(270,170, 490, 240);
				pros::screen::set_pen(0x00FFFFFF);
				pros::screen::print(pros::E_TEXT_MEDIUM, 30,25, "Red Positive"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 30,195, "Red Negative"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 310,25, "Blue Positive"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 310,195, "Blue Negative"); 
				status = pros::screen::touch_status();
				if (status.x > 0 && status.x < 200 && status.y > 0 && status.y < 70) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 30,25, "Selected     "); 
					autonSelection = 1;
					colourSortingSelector = 2;
					break;
				}
				else if (status.x > 0 && status.x < 200 && status.y > 170 && status.y < 240) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 30,195, "Selected     "); 
					autonSelection = 2;
					colourSortingSelector = 2;
					break;
				}
				else if (status.x > 270 && status.x < 490 && status.y > 0 && status.y < 70) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 310,25, "Selected     "); 
					autonSelection = 3;
					colourSortingSelector = 1;
					break;
				}
				else if (status.x > 270 && status.x < 490 && status.y > 170 && status.y < 240) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 310,195, "Selected     "); 
					autonSelection = 4;
					colourSortingSelector = 1;
					break;
				}
			}
			break;
		}
		else if (status.x > 200 && status.x < 340 && status.y > 70 && status.y < 170) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 235,110, "Selected"); 
			pros::delay(1000);
			pros::screen::set_eraser(0x00000000);
			pros::screen::erase();
			while (true) {
				pros::delay(30);
				pros::screen::set_pen(0x00FF0000);
				pros::screen::fill_rect(0,0,200,70);
				pros::screen::fill_rect(0,170, 200, 240);
				pros::screen::set_pen(0x000000FF);
				pros::screen::fill_rect(270,0,490,70);
				pros::screen::fill_rect(270,170, 490, 240);
				pros::screen::set_pen(0x00FFFFFF);
				pros::screen::print(pros::E_TEXT_MEDIUM, 30,25, "Red Positive"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 30,195, "Red Negative"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 310,25, "Blue Positive"); 
				pros::screen::print(pros::E_TEXT_MEDIUM, 310,195, "Blue Negative"); 
				status = pros::screen::touch_status();
				if (status.x > 0 && status.x < 200 && status.y > 0 && status.y < 70) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 30,25, "Selected     "); 
					autonSelection = 5;
					colourSortingSelector = 2;
					break;
				}
				if (status.x > 0 && status.x < 200 && status.y > 170 && status.y < 240) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 30,195, "Selected     "); 
					autonSelection = 6;
					colourSortingSelector = 2;
					break;
				}
				if (status.x > 270 && status.x < 490 && status.y > 0 && status.y < 70) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 310,25, "Selected     "); 
					autonSelection = 7;
					colourSortingSelector = 1;
					break;
				}
				if (status.x > 270 && status.x < 490 && status.y > 170 && status.y < 240) {
					pros::screen::print(pros::E_TEXT_MEDIUM, 310,195, "Selected     "); 
					autonSelection = 8;
					colourSortingSelector = 1;
					break;
				}
			}
			break;
		}
		else if (status.x > 400 && status.x < 480 && status.y > 0  && status.y < 240) {
			pros::screen::print(pros::E_TEXT_MEDIUM, 395,110, "Selected"); 
			autonSelection = 9;
			colourSortingSelector = 0;
			break;
		}
	}
	pros::Task Task([]{
        while (true) {
            liftControl();
			checkColour();
			colourSort();
			std::cout <<ringColour << " " <<colourSortingSensor.get_hue()<<" "<< ejectCounter << " " << colourSortingSensor.get_proximity() << " " <<colourSortingEjectionState <<" "<<colourSortingSelector<<"\n";
			pros::delay(20);
        }
    });
	// pros::Task colourSortTask([]{
	// 	while (true) {
			
	// 		pros::delay(20);
	// 	}
	// });
}
void disabled() {}

void competition_initialize() {}

void autonomous() { 
	if (autonSelection == 1) {
		qualsPosRed();
	}
	else if (autonSelection == 2) {
		qualsNegRed();	
	}
	else if (autonSelection == 3) {
		qualsPosBlue();	
	}
	else if (autonSelection == 4) {
		qualsNegBlue();
	}
	else if (autonSelection == 5) {
		elimsPosRed();
	}
	else if (autonSelection == 6) {
		elimsNegRed();
	}
	else if (autonSelection == 7) {
		elimsPosBlue();
	}
	else if (autonSelection == 8) {
		elimsNegBlue();
	}
	else if (autonSelection == 9) {
		skillsAuton();
	}
}

void opcontrol()
{
	//chassis.setPose(0,0,0);
	//driveToDistance(10, 127, false);
	//multitaskIntakeControl.suspend();
	leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
	wallStake.set_brake_mode(MOTOR_BRAKE_BRAKE);
	secondStageIntake.set_brake_mode(MOTOR_BRAKE_COAST);
	pros::screen::erase();
    while (true) {
		pros::delay(20);
		mogoMechControls();
		drivetrainControls();
		intakeControls();
		//mogoStealControls();
		// wallStakeControls();
		newWallStake();
		allianceStakeControls();
		lbMiddleTipControl();
		doinkerControls();
		lbTransport();
		lbTipControls();
		allianceAutoScore();
		lbMogoScoreControl();
		lemlib::Pose pose = chassis.getPose();
		pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
		pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
		pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    }
}