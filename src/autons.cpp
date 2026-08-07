#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstdint>
#include <future>
#include "globals.hpp"
#include "intake.cpp"
#include "piston.cpp"
#include "auton_functions.cpp"
#include "other_functions.cpp"
#include "pros/motors.h"

inline void skillsAuton() {
    chassis.setPose(-60.5, 9, -128);
    isNeutralGoal = false;
    isAutoReset = true;
    isAutonSkills = true;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    turnToPoint(-55, -21, false);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    moveDrive(-70, 500);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    moveToPoint(-48, -5, true, false);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-28, -23, false, true, true);
    leftMotors.move(-12);
	rightMotors.move(-12);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV - .25, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);

    //PICKUS UP 1st ground Pin

    // elevator.goToPinHeight(1);
    elevator.moveTo(ElevatorConfig::PIN_1_ELEV-2.52-.5, ElevatorConfig::PIN_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
    pros::delay(200);
    moveToPoint(-42, -24, false, true);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(450);
    goalReset();
    isAutoReset = true;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 350);
    pros::delay(1250);
    elevator.elevatorReset();
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-44, -44, false, true, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(2);
    //since elevator is off this is the right height for pin heihgt 3 alliance goal
    pros::delay(200);
    moveToPoint(-48, -27, false, true, true);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);

    skillsAquireAllianceMatchload();
    elevator.goToPinHeight(3);
    skillsScoreOnAllianceGoal();

    // skillsAquireAllianceMatchload();
    // elevator.goToPinHeight(4);
    // skillsScoreOnAllianceGoal();
    intake.setState(IntakeState::IDLE);

    moveToPoint(-48, -45, true, false, true);
    matchloadMode = false;
    intakeLift.setState(false);
    hood.setState(false);
    moveToPoint(-35, -31, false, false, true);
    moveToPoint(-35, -15, false, false, true);
    moveToPoint(-30,-4, false, false, true);
    intakeLift.setState(true);
    moveToPoint(-30, 12, true, true);
    intakeLift.setState(false);
    intake.setState(IntakeState::INTAKE);
    pros::delay(1000);
    turnToPoint(-56.5, 2, false);
    chassis.moveToPoint(-56.5, 2, 1000, {.forwards = false});
    chassis.waitUntil(6);
    isNeutralGoal = true;
    elevator.setState(ElevatorState::PICKUP);
    

    // moveToPoint(-48, 2, false, true);
    turnToHeading(180, true);
    moveDrive(-60, 500);
    elevator.goToPinHeight(0);
    pros::delay(350);
    elevator.setState(ElevatorState::SCORE);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(400);
    goalReset();
    moveToPoint(-60.5, 1, true, false, true);
    moveToPoint(-60.5, 30, false, true, true);
    moveToPoint(-45, 60, false, false, true);
    matchloadMode = true;
    intakeLift.setState(true);
    isAutoReset = true;
    hood.setState(true);
    turnToHeading(-90, false);
    rightResetPos();
    moveToPoint(-58, 56, true, true);
    intake.setState(IntakeState::INTAKE);
    elevator.setState(ElevatorState::MATCHLOAD);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    turnToHeading(-90, true);
    moveDrive(30, 750);
	moveDrive(-12, 150);
    isNeutralGoal = true;
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2750) {
            break;
        }
    }
    isNeutralGoal = true;
    elevator.goToPinHeight(1);
    skillsScoreOnNeutralGoal();

    skillsAcquireNeutralMatchload();

    elevator.goToPinHeight(2);
    skillsScoreOnNeutralGoal();
    isDropDownIntake = false;
    
    intake.setState(IntakeState::IDLE);
    isNeutralGoal = false;
    matchloadMode = false;
    goalReset();
	turnToHeading(0, true);
    moveDrive(50, 300);
    moveDrive(10, 50);
    turnToPoint(-27, 33, true);
    chassis.moveToPoint(-27, 33, 1000, {.maxSpeed = 60,.minSpeed = 10, .earlyExitRange = 5});
    chassis.waitUntil(7);
    intakeLift.setState(true);
    chassis.waitUntilDone();
	leftMotors.move(50);
	rightMotors.move(-5);
	pros::delay(150);
	intakeLift.setState(false);
    isDropDownIntake = true;
	intake.setState(IntakeState::INTAKE);
	pros::delay(550);
	moveDrive(5, 100);
	pros::delay(700);
	turnToPoint(-2, 29, false);
	chassis.moveToPoint(-2, 29, 1000, {.forwards = false, .earlyExitRange = 5});
	chassis.waitUntil(7);
	elevator.setState(ElevatorState::PICKUP);
    chassis.moveToPoint(0, 7, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    chassis.waitUntil(10);
    elevator.moveTo(3, -84, false);
    elevator.setState(ElevatorState::HOLD);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(450);
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(350);
    goalReset();
    moveDrive(50, 300);
	elevator.setState(ElevatorState::GROUND_PICKUP);
	moveDrive(10, 50);
    moveToPoint(19, 21, false, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV - 1, ElevatorConfig::GROUND_PICKUP_WRIST, true);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = false;
	elevator.moveTo(10.5, ElevatorConfig::PIN_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
    pros::delay(200);
    moveToPoint(6, 4, false, true, true);
    leftMotors.move(-55);
    rightMotors.move(-55);
    pros::delay(500);
    elevator.setState(ElevatorState::SCORE);
}

inline void skillsDriverMacro() {
    chassis.setPose(-60.5, 9, -128);
    isNeutralGoal = false;
    isAutoReset = true;
    isAutonSkills = true;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    turnToPoint(-55, -21, false);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    moveDrive(-70, 500);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    elevator.setState(ElevatorState::SCORE);
    if (skillsTask != nullptr) {
        skillsTask->suspend();
        isSkillsMacro = false;
	}
    // goalReset();
    // pros::delay(350);
    // moveToPoint(-48, -5, true, false, true);
    // elevator.setState(ElevatorState::GROUND_PICKUP);
    // moveToPoint(-28, -23, false, true, true);
    // leftMotors.move(-12);
	// rightMotors.move(-12);
    // elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV - .25, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    // elevator.setState(ElevatorState::HOLD);
	// pros::delay(500);
    // clamp.setState(true);
	// pros::delay(200);
    // elevator.setState(ElevatorState::PIN_HEIGHT);

    // //PICKUS UP 1st ground Pin

    // // elevator.goToPinHeight(1);
    // elevator.moveTo(ElevatorConfig::PIN_1_ELEV-2.52-.5, ElevatorConfig::PIN_WRIST, false);
    // elevator.setState(ElevatorState::HOLD);
    // pros::delay(200);
    // moveToPoint(-42, -24, false, true);
    // leftMotors.move(-35);
    // rightMotors.move(-35);
    // pros::delay(450);
    // goalReset();
    // isAutoReset = true;
    // elevator.setState(ElevatorState::SCORE);
    // pros::delay(300);
    // moveDrive(50, 350);
    // pros::delay(1250);
    // elevator.elevatorReset();
    // elevator.setState(ElevatorState::GROUND_PICKUP);
    // moveToPoint(-44, -44, false, true, true);
    // leftMotors.move(-15);
	// rightMotors.move(-15);
    // elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    // elevator.setState(ElevatorState::HOLD);
	// pros::delay(500);
    // clamp.setState(true);
	// pros::delay(200);
    // elevator.setState(ElevatorState::PIN_HEIGHT);
    // elevator.goToPinHeight(2);
    // //since elevator is off this is the right height for pin heihgt 3 alliance goal
    // pros::delay(200);
    // moveToPoint(-48, -27, false, true, true);
    // leftMotors.move(-35);
    // rightMotors.move(-35);
    // pros::delay(300);
    // elevator.setState(ElevatorState::SCORE);
    // goalReset();
    // pros::delay(350);
    // matchloadMode = true;
    // intakeLift.setState(true);
    // hood.setState(true);
    // leftMotors.move(50);
    // rightMotors.move(50);
    // if (skillsTask != nullptr) {
    //     skillsTask->suspend();
    //     isSkillsMacro = false;
	// }
}

inline void autonFirstPartLeft() {
    chassis.setPose(-60.5, 9, -128);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(-25, 300, {.minSpeed = 80});
    chassis.moveToPoint(-62, -10, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    chassis.swingToHeading(-93.5, DriveSide::RIGHT, 700, {.minSpeed = 90, .earlyExitRange = 8});
    chassis.waitUntilDone();
    moveDrive(0, 200);
    leftMotors.move(-10);
    rightMotors.move(-10);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(300);
    isFastReset = true;
    elevator.setState(ElevatorState::RESET);
    turnToHeading(-90);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	moveDrive(45, 400);
	moveDrive(-20, 200);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 1800) {
            break;
        }
        if(pros::millis() - startTime > 1500) {
            hood.setState(false);
        }
    }
    if(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        elevator.setState(ElevatorState::PICKUP);
    }
    isFastReset = false;
    turnToHeading(-80, true);
    moveDrive(-60, 500);
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    elevator.setState(ElevatorState::RESET);
    chassis.moveToPoint(-63, -10, 700, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-63, -30, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
  //  chassis.swingToHeading(-153, DriveSide::LEFT,  1000, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.waitUntilDone();
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-50, -46, false, true, true);
    leftMotors.move(-10);
	rightMotors.move(-10);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 

    // PICKS UP FIRST GROUND PIN
}
inline void autonFirstPartRight() {
    chassis.setPose(9, -60.5, 218);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(155, 300, {.minSpeed = 80});
    chassis.moveToPoint(-9, -62.5, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    chassis.swingToHeading(180, DriveSide::LEFT, 700, {.minSpeed = 90, .earlyExitRange = 10});
    chassis.waitUntilDone();
    elevator.setState(ElevatorState::SCORE);
    moveDrive(0, 200);
    leftMotors.move(-10);
    rightMotors.move(-10);
    isFastReset = true;
	goalReset();
    elevator.setState(ElevatorState::RESET);
	goalJiggle(180, 300);
    goalJiggle(180, 500);
    // pros::delay(300);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	moveDrive(25, 550);
	moveDrive(-20, 200);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2000) {
            break;
        }
    }
    isFastReset = true;
    moveDrive(-60, 500);
    isAutoReset = true;
    elevator.setState(ElevatorState::SCORE);
    intake.setState(IntakeState::OUTTAKE);
    goalReset();
    pros::delay(350);
    // elevator.setState(ElevatorState::RESET);
    chassis.moveToPoint(-10, -63, 700, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-30, -63, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    elevator.setState(ElevatorState::GROUND_PICKUP);
    isFastReset = false;
    moveToPoint(-47, -49, false, true, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
    intake.setState(IntakeState::IDLE);
    leftMotors.move(0);
    rightMotors.move(0);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 

    // PICKS UP FIRST GROUND PIN
}
inline void SAWP() {

}

inline void TAWPLeft() { // teamate has to score one pin and get toggle
    autonFirstPartLeft();
    // PICKS UP FIRST GROUND PIN

    elevator.goToPinHeight(1);
    pros::delay(200);
    chassis.moveToPoint(-27, -47, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-25, -28, false, true, true);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV-.4, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    // elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = true; 
    // elevator.goToPinHeight(1);
    isAutoReset = true;
    elevator.moveTo(ElevatorConfig::PIN_1_ELEV - 1.5, ElevatorConfig::PIN_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
    pros::delay(200);
    chassis.turnToPoint(-42, 18, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-42, 18, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    elevator.setState(ElevatorState::SCORE);
}

inline void TAWPRight() {
    autonFirstPartRight();

    elevator.goToPinHeight(1);
    pros::delay(200);
    chassis.moveToPoint(-48, -27, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-28, -24.5, false, true, true);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = true; 
    elevator.goToPinHeight(1);
    isAutoReset = true;
    pros::delay(200);
    chassis.turnToPoint(20, -42, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(20, -42, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    elevator.setState(ElevatorState::SCORE);
}

inline void elimsAllianceGoalLeft() {
    autonFirstPartLeft();

    elevator.goToPinHeight(2);
    pros::delay(200);
    chassis.turnToPoint(-41, -27, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-41, -27, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-29, -24, false, true, true);
    elevator.moveTo(-5, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(3);
    // since elevator is off this is the right height for pin heihgt 3 alliance goal
    isAutoReset = true;
    pros::delay(200);
    chassis.turnToPoint(-45, -21, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-45, -21, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    elevator.setState(ElevatorState::SCORE);

}



inline void elimsAllianceGoalRight() {
    
    autonFirstPartRight();

    elevator.goToPinHeight(2);
    pros::delay(200);
    chassis.turnToPoint(-27, -41, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-27, -41, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-24, -29, false, true, true);
    elevator.moveTo(-5, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(3);
    // since elevator is off this is the right height for pin heihgt 3 alliance goal
    isAutoReset = true;
    pros::delay(200);
    chassis.turnToPoint(-21, -45, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-21, -45, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    elevator.setState(ElevatorState::SCORE);
}




inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
    }
    if(startingSide == Selection::LEFT) {
        if(autonSelectionEnum == Selection::TAWP) {
            TAWPLeft();
        }
        if(autonSelectionEnum == Selection::ELIMS) {
            elimsAllianceGoalLeft();
        }
        if(autonSelectionEnum == Selection::CORNER) {

        }
        if(autonSelectionEnum == Selection::NOTHING) {

        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::TAWP) {
            TAWPRight();
        }
        if(autonSelectionEnum == Selection::ELIMS) {
            // rightLGAndC_CenterLast();
            elimsAllianceGoalRight();
        }
        if(autonSelectionEnum == Selection::CORNER) {

        }
        if(autonSelectionEnum == Selection::NOTHING) {

        }
    }
}