#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "motor_functions.hpp"
#include "pros/rtos.hpp"
#include <type_traits>

void testing_auton() {

}

void nonMogoRushRed() {
	chassis.setPose(-60, 17, -180);
	chassis.moveToPoint(-60, 0, 3000);
	intake.move(-127);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	chassis.moveToPoint(-60, 13, 500, {.forwards = false});
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.swingToPoint(-70, 0, lemlib::DriveSide::LEFT, 1000, {.forwards = false});
	chassis.moveToPoint(-64.5, 1, 700, {.forwards = false});
	chassis.waitUntilDone();
	spinIntake(90);
	pros::delay(400);
	chassis.moveToPoint(-61, 1, 1000);
	chassis.moveToPoint(-66, 1, 1000, {.forwards = false, .minSpeed = 100});
	chassis.moveToPoint(-44, 24, 1000);
	chassis.turnToPoint(-24, 24, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	stopIntake();
	chassis.moveToPoint(-18, 24, 2000, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntil(22);
	mogoMech.set_value(true);
	chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	spinIntake(127);
	// starting to get the the two in the center
	chassis.moveToPoint(-24, 52, 2000);
	chassis.turnToPoint(-9, 58, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(-9, 58, 2000);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.moveToPoint(-18, 48, 2000, {.forwards = false});
	chassis.moveToPoint(-9, 50, 2000);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.swingToHeading(-160, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 20, .earlyExitRange = 40});
	chassis.moveToPoint(-16, 23, 2000);
	chassis.waitUntil(24);
	wallStake.move(127);
}

void mogoRushRedClearCorner() {
	chassis.setPose(-52, -36, 90);
	chassis.moveToPose(-8.5, -58.5, 120, 1500, {.minSpeed = 127});
	intake.move(127);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	pros::delay(100);
	chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 1000);
	chassis.waitUntilDone();
	intake.move(0);
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	//chassis.turnToHeading(180, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 20});
	chassis.turnToHeading(180, 1000,{.maxSpeed = 60, .earlyExitRange = 40});
	chassis.moveToPoint(-12, -42, 1000, {.forwards = false});
	chassis.waitUntilDone();
	mogoMech.set_value(true); 
	pros::delay(250);
	spinIntake(127);
	chassis.moveToPoint(-45, -60, 1000, {.earlyExitRange = 20});
	chassis.waitUntilDone();
	hook.move(0);
	intake.move(-127);
	chassis.moveToPose(-72, -56, -90, 1000);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	stopIntake();
	pros::delay(500); 
	chassis.moveToPoint(-55, -56, 1000, {.forwards = false});
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.turnToHeading(180, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	mogoMech.set_value(false);
	chassis.turnToHeading(-135, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.moveToPoint(-24, -24, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	mogoMech.set_value(true); 
	pros::delay(500);
	chassis.moveToPoint(-10, -10, 1000);
	chassis.waitUntilDone();
	wallStake.move(127);
}

void mogoRushRedCarry() {

}

/*void nonMogoRushBlue() {
	chassis.setPose(55, 24, 90);
	chassis.moveToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(300);
	chassis.turnToPoint(9.5, 36, 1500, {.earlyExitRange = 25});
	chassis.waitUntilDone();
	spinIntake(127);
	chassis.moveToPose(9.5, 52, 0, 2000);
	chassis.moveToPoint(24, 24,  1500, {.forwards = false});
	chassis.turnToPoint(24, 48, 1000, {.earlyExitRange = 25});
	chassis.moveToPoint(24, 48, 1500, {.earlyExitRange = 5});
	chassis.waitUntilDone();
	stopIntake();
	chassis.turnToPoint(56, 12, 1500, {.earlyExitRange = 20});
	chassis.waitUntilDone();
	chassis.moveToPose(56, -5, 180, 1500);
	holdPosition = false;
	positionSetting = true;	
	while(!loadStake) {
		loadStake = loadWallStake();
	}
}*/

void nonMogoRushBlue() {
	chassis.setPose(56, 13.5, -90);
	chassis.moveToPoint(48, 12, 1000);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	chassis.moveToPoint(57, 12, 500, {.forwards = false, .maxSpeed = 50});
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	pros::delay(500);
	chassis.swingToPoint(55, 0, lemlib::DriveSide::RIGHT, 800);
	intake.move(127);
	chassis.moveToPoint(58, 0, 1000);
	chassis.turnToPoint(64, 0, 800, {.forwards = false});
	chassis.waitUntilDone();
	intake.move(0);
	chassis.moveToPoint(64, -1, 800, {.forwards = false});
	chassis.waitUntilDone();
	spinIntake(127);
	pros::delay(600);
	stopIntake();
	chassis.moveToPoint(48, 24, 1500);
	chassis.turnToPoint(24, 24, 800, {.forwards = false, .earlyExitRange = 30});
	chassis.moveToPoint(18, 24, 2000, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntil(26);
	// pick up mogo
	mogoMech.set_value(true);
	chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	spinIntake(127);
	// starting to get the the two in the center
	chassis.moveToPoint(24, 52, 2000);
	chassis.turnToPoint(9, 57, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(9, 57, 2000);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.moveToPoint(22, 48, 2000, {.forwards = false});
	chassis.moveToPoint(9, 49, 2000);
	chassis.waitUntilDone();
	pros::delay(300);
	chassis.swingToHeading(160, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 20, .earlyExitRange = 40});
	chassis.moveToPoint(16, 18, 2000);
	chassis.waitUntil(26);
	wallStake.move(127);
}

void mogoRushBlue() {
	chassis.setPose(52, -60, -90);
	chassis.moveToPose(11, -38, -54, 1500, {.minSpeed = 127});
	intake.move(127);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	pros::delay(100);
	chassis.swingToHeading(-135, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 90});
	chassis.waitUntilDone();
	intake.move(0);
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.turnToHeading(0, 1000, {.maxSpeed = 50, .earlyExitRange = 30});
	chassis.waitUntilDone();
	chassis.moveToPoint(15, -63, 1000, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(200);
	spinIntake(127);
	pros::delay(500);
	stopIntake();
	mogoMech.set_value(false);
	chassis.turnToPoint(24, -24, 1500, {.forwards = false, .earlyExitRange = 30});
	chassis.moveToPoint(25, -20, 1500, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntil(34);
	mogoMech.set_value(true);
	pros::delay(200);
	spinIntake(127);
	chassis.moveToPoint(57, -24, 1000, {.earlyExitRange = 15});
	chassis.waitUntilDone();
	stopIntake();
	chassis.moveToPoint(55, -66, 1500);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	pros::delay(700);
	chassis.moveToPoint(54, -24, 1000, {.forwards = false, .earlyExitRange = 10});
	diggerMech.set_value(false);
	chassis.turnToPoint(24, 0, 1000, {.earlyExitRange = 15});
	chassis.moveToPoint(24, 0, 1500, {.maxSpeed = 70});
}


	 
	// mogoMech.set_value(false);
	// chassis.waitUntilDone();
	// spinIntake(127);
	// chassis.turnToPoint(43, -5, 1500, {.earlyExitRange = 15});
	// chassis.moveToPoint(50, 0, 1500, {.maxSpeed = 40});
	// chassis.waitUntilDone();
	// chassis.turnToHeading(-90, 0, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	// stopIntake();
	// chassis.moveToPoint(60, 0, 1300, {.forwards = false});

void skillsAuton() {
   chassis.setPose(-62, 0, 90);
   spinIntake(127);
   pros::delay(400);
   stopIntake();
   chassis.swingToPoint(-52, -18, lemlib::DriveSide::LEFT, 1500, {.forwards = false, .minSpeed = 80, .earlyExitRange = 40});
   chassis.moveToPose(-47, -29, -30, 1600, {.forwards = false, .maxSpeed = 70, .earlyExitRange = 5});
   chassis.waitUntilDone();
   mogoMech.set_value(true);
   pros::delay(500);
   spinIntake(127);
   chassis.moveToPoint(-24, -24, 1500, {.earlyExitRange = 10});
   chassis.moveToPoint(-2, -58, 1500, {.earlyExitRange = 10});
   chassis.turnToPoint(-24, -48, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, -48, 3000, {.maxSpeed = 70, .earlyExitRange = 10});
   chassis.moveToPoint(-60, -48, 3000, {.maxSpeed = 70, .earlyExitRange = 10});
   chassis.swingToPoint(-48, -60, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-48, -60, 3000, {.maxSpeed = 70,.earlyExitRange = 10});
   chassis.turnToHeading(45, 1000);
   chassis.waitUntilDone();
   mogoMech.set_value(false);
   chassis.moveToPoint(-60, -60, 1500, {.forwards = false});
   stopIntake();
   chassis.moveToPoint(-48, 0, 1500);
   chassis.turnToPoint(-48, 24, 1500, {.forwards = false, .earlyExitRange = 20});
   chassis.moveToPoint(-48, 21, 1500, {.forwards = false, .maxSpeed = 60});
   chassis.waitUntilDone();
   mogoMech.set_value(true);
   pros::delay(700);
   spinIntake(127);
   chassis.turnToPoint(-24, 24, 1500, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, 24, 1500, {.earlyExitRange = 10});
   chassis.moveToPoint(-2, 58, 1500);
   chassis.turnToPoint(-24, 48, 1500, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, 48, 1500, {.maxSpeed = 60,.earlyExitRange = 10});
   chassis.moveToPoint(-57, 48, 1500, {.maxSpeed = 60,.earlyExitRange = 10});
   chassis.swingToPoint(-48, 60, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-48, 60, 1500, {.earlyExitRange = 10});
   chassis.turnToHeading(135, 1000, {.maxSpeed = 60,.earlyExitRange = 20});
   chassis.waitUntilDone();
   mogoMech.set_value(false);
   chassis.moveToPoint(-60, 60, 1500, {.forwards = false});
   stopIntake();
   intake.move(127);
   chassis.moveToPoint(24, 48, 2000);
   hook.move(50);
   chassis.moveToPoint(24, 24, 1500, {.earlyExitRange = 10});
   chassis.waitUntilDone();
   stopIntake();
   chassis.turnToPoint(48, 0, 1500, {.forwards = false, .earlyExitRange = 20});
   chassis.moveToPoint(44, 4, 1500, {.forwards = false, .maxSpeed = 60});
   mogoMech.set_value(false);
   pros::delay(500);
   chassis.turnToPoint(24, -24, 1500, {.earlyExitRange = 20});
   chassis.waitUntilDone();
   spinIntake(127);
   chassis.moveToPoint(24, -24, 1000, {.earlyExitRange = 10});
   chassis.moveToPoint(24, -45, 1000, {.earlyExitRange = 10});
   chassis.turnToPoint(48, -48, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(55, -48, 1000, {.maxSpeed = 30,.earlyExitRange = 10});
   chassis.waitUntilDone(); 
   chassis.swingToHeading(-10, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 60,.earlyExitRange = 20});
   chassis.waitUntilDone();
   mogoMech.set_value(false);
   pros::delay(500);
}
