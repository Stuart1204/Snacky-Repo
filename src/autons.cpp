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

void mogoRushRed() {
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
	chassis.turnToHeading(180, 1000, {.maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 20});
	chassis.moveToPoint(-19, -40, 1500, {.forwards = false});
	chassis.waitUntilDone();
	mogoMech.set_value(true);
	pros::delay(250);
	spinIntake(127);
	chassis.moveToPoint(-45, -60, 1000, {.earlyExitRange = 20});
	chassis.waitUntilDone();
	intake.move(-127);
	chassis.moveToPose(-67.25, -56, -90, 1000);
	chassis.waitUntilDone();
	diggerMech.set_value(true);
	pros::delay(500); 
	chassis.moveToPoint(-55, -56, 1000, {.forwards = false});
	chassis.waitUntilDone();
	diggerMech.set_value(false);
	chassis.turnToHeading(45, 1000, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
}