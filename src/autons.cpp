#include "autons.hpp"
#include "lemlib/timer.hpp"

void testing_auton() {
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
}