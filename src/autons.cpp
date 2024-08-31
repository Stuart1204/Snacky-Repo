#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/timer.hpp"
#include "motor_functions.hpp"
#include "pros/rtos.hpp"
#include <type_traits>

void testing_auton() {

}

void nonMogoRushRed() {
	chassis.setPose(-59, 17, -180);
	chassis.moveToPoint(-59, 0, 3000);
	intake.move(-127);
	chassis.turnToHeading(90, 1000);
	chassis.moveToPoint(-64, 0, 500, {.forwards = false});
	chassis.waitUntilDone();
	spinIntake(90);
	pros::delay(200);
	chassis.moveToPoint(-58, 0, 1000);
	chassis.moveToPoint(-66, 0, 700, {.forwards = false, .minSpeed = 80});
	chassis.moveToPoint(-48, 0, 2000);
	chassis.swingToHeading(45, lemlib::DriveSide::LEFT, 1000);
	chassis.waitUntilDone();
	chassis.moveToPoint(-36, 12, 1500);
	chassis.turnToHeading(-135, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	while(ringTracker.get_hue() > 70) {
		spinIntake(80);
	}
	stopIntake();
	chassis.waitUntilDone();
	chassis.moveToPoint(-22, 26, 2000, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(200);
	chassis.turnToHeading(0, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	spinIntake(127);
	chassis.moveToPoint(-24, 52, 2000);
	chassis.turnToPoint(-7, 54, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(-7, 54, 2000);
	chassis.moveToPoint(-16, 45, 2000, {.forwards = false});
	chassis.moveToPoint(-6, 45, 2000);
	chassis.turnToHeading(-135, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(-16, 18, 2000);
	chassis.waitUntilDone();
	wallStake.move(127);
}

void mogoRushRedClearCorner() {
	chassis.setPose(-52, -36, 90);
	chassis.moveToPose(-10, -57.25, 120, 1500, {.minSpeed = 127});
	intake.move(127);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 1000);
	chassis.waitUntilDone();
	intake.move(0);
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	//chassis.turnToHeading(180, 1000, {.maxSpeed = 60, .minSpeed = 20, .earlyExitRange = 20});
	chassis.swingToHeading(180, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 60, .minSpeed = 20});
	chassis.moveToPoint(-19, -43, 1500, {.forwards = false, .maxSpeed = 60});
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
	pros::delay(500); 
	chassis.moveToPoint(-55, -56, 1000, {.forwards = false});
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.turnToHeading(45, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.waitUntilDone();
	mogoMech.set_value(false);
	pros::delay(500);
	stopIntake();
	chassis.moveToPoint(-31, -33, 2000, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	mogoMech.set_value(true); 
	pros::delay(500);
	spinIntake(127);
	chassis.moveToPose(-10, -31, 90, 1000);
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
	chassis.setPose(59, 16, 180);
	chassis.moveToPoint(58, 0, 3000);
	intake.move(-127);
	chassis.turnToHeading(-90, 1000);
	chassis.moveToPoint(62, 0, 500, {.forwards = false, .maxSpeed = 80});
	chassis.waitUntilDone();
	spinIntake(100);
	pros::delay(600);
	chassis.moveToPoint(60, 0, 1000);
	chassis.waitUntilDone();
	stopIntake();
	chassis.moveToPoint(64, 0, 700, {.forwards = false, .minSpeed = 90});
	spinIntake(127);
	chassis.moveToPoint(48, 0, 2000);
	chassis.swingToHeading(-45, lemlib::DriveSide::RIGHT, 1000);
	chassis.waitUntilDone();
	chassis.moveToPoint(36, 12, 1500);
	chassis.turnToHeading(135, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	while(ringTracker.get_hue() < 70 ) {
		hook.move(80);
		intake.move(127);
	}
	stopIntake();
	chassis.waitUntilDone();
	chassis.moveToPoint(22, 26, 2000, {.forwards = false, .maxSpeed = 60});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(200);
	chassis.turnToHeading(180, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	spinIntake(127);
	chassis.moveToPoint(24, 48, 2000);
	chassis.turnToPoint(7, 53, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(7, 53, 2000);
	chassis.moveToPose(18, 43, -40, 2000, {.forwards = false, .minSpeed = 80,.earlyExitRange = 10});
	chassis.moveToPoint(7, 40, 2000);
	chassis.turnToHeading(-35, 1000, {.minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(16, 18, 2000, {.forwards = false});
	chassis.waitUntilDone();
	wallStake.move(127);
}

void mogoRushBlue() {
	chassis.setPose(52, -60, -90);
	chassis.moveToPose(8, -38, -60, 1500, {.minSpeed = 127});
	intake.move(127);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	chassis.swingToHeading(-135, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 90});
	chassis.waitUntilDone();
	intake.move(0);
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.swingToHeading(85, lemlib::DriveSide::LEFT, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE, .minSpeed = 20, .earlyExitRange = 20});
	chassis.waitUntilDone();
	chassis.moveToPose(16, -57, 80, 1000, {.forwards = false});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(200);
	spinIntake(127);
	pros::delay(600);
	stopIntake();
	mogoMech.set_value(false);
	pros::delay(200);
	chassis.turnToPoint(24, -24, 1000, { .forwards = false, .direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .earlyExitRange = 15});
	chassis.moveToPoint(24, -20, 1500, {.forwards = false});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	spinIntake(127);
	pros::delay(900);
	stopIntake();
	chassis.waitUntilDone();
	intake.move(-127);
	chassis.moveToPose(52, -71, 180, 3000);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	hook.move(-127);
	pros::delay(400);
	hook.move(127);
	chassis.waitUntilDone();
	chassis.moveToPoint(54, -45, 1000, {.forwards = false});
	chassis.waitUntilDone();
	diggerMech.set_value(false); 
	chassis.turnToHeading(0, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
	chassis.moveToPoint(54, -58, 1000, {.forwards = false});
	mogoMech.set_value(false);
	chassis.moveToPoint(54, -51, 1000);
	chassis.waitUntilDone(); 
	chassis.turnToHeading(90, 1000, {.earlyExitRange = 20});

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
   chassis.swingToPoint(-52, -18, lemlib::DriveSide::LEFT, 1500, {.forwards = false, .earlyExitRange = 20});
   chassis.moveToPoint(-52, -18, 1000, {.forwards = false, .maxSpeed = 60});
   chassis.waitUntilDone();
   mogoMech.set_value(true);
   pros::delay(700);
   spinIntake(127);
   chassis.moveToPoint(-24, -24, 1500, {.earlyExitRange = 10});
   chassis.moveToPoint(-2, -58, 1500, {.earlyExitRange = 10});
   chassis.turnToPoint(-24, -48, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, -48, 3000, {.maxSpeed = 70, .earlyExitRange = 10});
   chassis.moveToPoint(-57, -48, 3000, {.maxSpeed = 70, .earlyExitRange = 10});
   chassis.swingToPoint(-48, -60, lemlib::DriveSide::RIGHT, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-48, -60, 3000, {.maxSpeed = 70,.earlyExitRange = 10});
   chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 70, .earlyExitRange = 20});
   chassis.waitUntilDone();
   pros::delay(250);
   mogoMech.set_value(false);
   stopIntake();
   chassis.moveToPoint(-60, -60, 1500, {.forwards = false});
   chassis.moveToPoint(-48, 0, 1500);
   chassis.turnToPoint(-48, 24, 1500, {.forwards = false, .earlyExitRange = 20});
   chassis.moveToPoint(-48, 21, 1500, {.forwards = false});
   chassis.waitUntilDone();
   mogoMech.set_value(true);
   pros::delay(700);
   spinIntake(127);
   chassis.turnToPoint(-24, 24, 1500, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, 24, 1500, {.earlyExitRange = 10});
   chassis.moveToPoint(-2, 58, 1500);
   chassis.turnToPoint(-24, 48, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-24, 48, 1500, {.maxSpeed = 30,.earlyExitRange = 10});
   chassis.moveToPoint(-57, 48, 1500, {.maxSpeed = 30,.earlyExitRange = 10});
   chassis.swingToPoint(-48, 60, lemlib::DriveSide::LEFT, 1000, {.maxSpeed = 60, .earlyExitRange = 20});
   chassis.moveToPoint(-48, 60, 1500, {.earlyExitRange = 10});
   chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE,.maxSpeed = 60,.earlyExitRange = 20});
   chassis.waitUntilDone();
   pros::delay(250);
   mogoMech.set_value(false);
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
