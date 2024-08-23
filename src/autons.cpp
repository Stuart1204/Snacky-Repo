#include "autons.hpp"
#include "lemlib/timer.hpp"
#include "motor_functions.hpp"
#include "pros/rtos.hpp"

void testing_auton() {
    chassis.setPose(0, 0, 0);
	/*chassis.moveToPose(-165, 0, 90, 2000);
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
	lemlib::Pose pose = chassis.getPose();*/
	chassis.turnToPoint(-24, -48, 2000);
	chassis.moveToPoint(18, 38, 3000, {.forwards=false, .minSpeed = 30, .earlyExitRange = 12});
	chassis.moveToPoint(23, 46, 3000, {.forwards=false, .maxSpeed = 30});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(200);
	chassis.moveToPoint(0, 96, 3000);
	runIntake();
	chassis.moveToPoint(24, 48, 3000);
	chassis.moveToPoint(0, 0, 3000);
	stopIntake();
	
	chassis.waitUntilDone();
}