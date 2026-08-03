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
    moveToPoint(-32, -30, false, false, true);
    moveToPoint(-36, -15, false, false, true);
    moveToPoint(-30,-4, false, false, true);
    intakeLift.setState(true);
    moveToPoint(-30, 10, true, true);
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
    pros::delay(300);
    elevator.setState(ElevatorState::SCORE);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(350);
    goalReset();
    moveToPoint(-61, 1, true, false, true);
    moveToPoint(-61, 30, false, true, true);
    moveToPoint(-46, 60, false, false, true);
    matchloadMode = true;
    intakeLift.setState(true);
    isAutoReset = true;
    hood.setState(true);
    turnToHeading(-90, false);
    rightResetPos();

    moveToPoint(-61, 56, true, true);
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
    
    intake.setState(IntakeState::IDLE);
    isNeutralGoal = false;
    matchloadMode = false;
    moveDrive(50, 250);
    intakeLift.setState(true);
    moveToPoint(-31, 25, true, true);
    intake.setState(IntakeState::INTAKE);
    intakeLift.setState(false);
    pros::delay(1000);
    elevator.setState(ElevatorState::PICKUP);
    intake.setState(IntakeState::OUTTAKE);
    moveToPoint(-24, 15, true, false, true);
    turnToPoint(0, 0, false);
    chassis.moveToPoint(-4, 4, 1000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntil(12);
    elevator.moveTo(5, -80, false);
    elevator.setState(ElevatorState::HOLD);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(450);
    isAutoReset = false;
    elevator.setState(ElevatorState::SCORE);
    pros::delay(350);
    goalReset();
    moveDrive(50, 300);
    moveToPoint(19, 23, false, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV - .5, ElevatorConfig::GROUND_PICKUP_WRIST, true);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(2);
    //since elevator is off this is the right height for pin heihgt 3 alliance goal
    pros::delay(200);
    moveToPoint(4, 4, false, true, true);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    elevator.setState(ElevatorState::SCORE);
    
}

inline void SAWP() {

}

inline void TAWP() { // teamate has to score one pin and get toggle
    chassis.setPose(-60.5, 9, -128);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(-25, 300, {.minSpeed = 80});
    chassis.moveToPoint(-62.5, -9, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    chassis.swingToHeading(-90, DriveSide::RIGHT, 700, {.minSpeed = 90, .earlyExitRange = 10});
    chassis.waitUntilDone();
    moveDrive(0, 200);
    leftMotors.move(-10);
    rightMotors.move(-10);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(300);
    isFastReset = true;
    elevator.setState(ElevatorState::RESET);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	moveDrive(45, 400);
	moveDrive(-20, 200);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2500) {
            break;
        }
    }
    isFastReset = false;
    turnToHeading(-80, true);
    moveDrive(-60, 500);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    elevator.setState(ElevatorState::RESET);
    chassis.moveToPoint(-63, -10, 700, {.minSpeed = 30, .earlyExitRange = 3});
    elevator.setState(ElevatorState::GROUND_PICKUP);
    chassis.moveToPoint(-63, -30, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
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

    elevator.goToPinHeight(1);
    pros::delay(200);
    chassis.moveToPoint(-27, -48, 700, {.forwards = false, .earlyExitRange = 4});
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
    moveToPoint(-25, -27, false, true, true);
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
    chassis.turnToPoint(-46, 20, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-46, 20, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    elevator.setState(ElevatorState::SCORE);
}

inline void elimsAllianceGoal() {
    chassis.setPose(-60.5, 9, -128);
    isNeutralGoal = false;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(-25, 300, {.minSpeed = 80});
    chassis.moveToPoint(-63, -10, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    chassis.swingToHeading(-90, DriveSide::RIGHT, 700, {.minSpeed = 90, .earlyExitRange = 10});
    chassis.waitUntilDone();
    moveDrive(0, 200);
    leftMotors.move(-10);
    rightMotors.move(-10);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    isFastReset = true;
    elevator.setState(ElevatorState::RESET);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	moveDrive(30, 250);
    pros::delay(300);
    moveDrive(30, 350);
	moveDrive(-20, 200);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2000) {
            break;
        }
    }
    isFastReset = false;
    turnToHeading(-80, true);
    moveDrive(-60, 500);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
    elevator.setState(ElevatorState::RESET);
    intake.setState(IntakeState::OUTTAKE);
    chassis.moveToPoint(-63, -10, 700, {.minSpeed = 30, .earlyExitRange = 3});
    elevator.setState(ElevatorState::GROUND_PICKUP);
    chassis.moveToPoint(-63, -30, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    moveToPoint(-50, -46, false, true, true);
    intake.setState(IntakeState::IDLE);
    leftMotors.move(-10);
	rightMotors.move(-10);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 

    // PICKUS UP 1st ground Pin

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



inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
        enableColourSort = false;
    }
    if(startingSide == Selection::LEFT) {
        // if(autonSelectionEnum == Selection::LONG_CONTROL) {
        //     // left7Block(); // not long control
        //     left7BlockWing();
        // }
        // if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
        //     // leftLGAndC_CenterLast();
        //     leftLGAndCRush();
        // }
        // if(autonSelectionEnum == Selection::SOLO_AWP) {
        //     midRush();
        // }
        // if(autonSelectionEnum == Selection::NI_SAWP) {
        //     upperMidRush();
        // }
    }
    if(startingSide == Selection::RIGHT) {
        // if(autonSelectionEnum == Selection::LONG_CONTROL) {
        //     // right7Block(); // not long control
        //     // right7BlockWing();
        //     right7Block();
        // }
        // if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
        //     // rightLGAndC_CenterLast();
        //     rightLGAndCRush();
        // }
        // if(autonSelectionEnum == Selection::SOLO_AWP) {
        //     SAWP13();
        // }
        // if(autonSelectionEnum == Selection::NI_SAWP) {
        //     SAWP4Goal();
        // }
    }
}