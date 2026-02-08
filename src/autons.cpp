#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <future>
#include "globals.hpp"
#include "intake.cpp"
#include "piston.cpp"
#include "auton_functions.cpp"
#include "other_functions.cpp"

inline void autonWingMacro() {
    startAutoWing();
    while(enableAutoWing) {
        autonAutoWing();
    }
    leftMotors.move(7);
    rightMotors.move(-7);
    pros::delay(300);
    leftMotors.move(-70);
    rightMotors.move(-60);
    pros::delay(500);
    leftMotors.move(-15);
    rightMotors.move(-30);
    pros::delay(1000);
    leftMotors.move(20);
    rightMotors.move(-35);
}

inline void autonWingMacroOpposite() {
    // startAutoWing();
    // while(enableAutoWing) {
    //     autonAutoWingOpposite();
    // }
    turnToHeading(-90, true);
    wing.setState(true);
	turnToHeading(30, false);
	turnToHeading(110, true);
	wing.setState(false);
    leftMotors.move(-7);
    rightMotors.move(7);
    pros::delay(100);
    turnToHeading(95, true);
    leftMotors.move(70);
    rightMotors.move(60);
    pros::delay(150);
    chassis.turnToHeading(75, 1500, {.maxSpeed = 50, .minSpeed = 30});
}

// grabs blocks under goal
inline void rightGrabBlocks() {
    chassis.setPose(-46.5, -8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -24, true, false, true, 65);
    tongue.setState(true);
    moveToPoint(-17, -37, true, false, true, 60);
    tongue.setState(false);
    moveToPoint(-9, -45, true, false, true, 80);
    tongue.setState(true);
    pros::delay(400);
}

// grabs blocks under goal
inline void leftGrabBlocks() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 24, true, false, true, 65);
    tongue.setState(true);
    moveToPoint(-17, 37, true, false, true, 60);
    tongue.setState(false);
    moveToPoint(-9, 45, true, false, true, 80);
    tongue.setState(true);
    pros::delay(400);
}

inline void rightAutoMatchload() { // grabs matchloads right away
    chassis.setPose(-48, -17.5, 180);
    resetPos();
    tongue.setState(true);
    moveToPoint(-48, -47.5, true, false, false);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    waitAtMatchload();
    leftResetPos();
}

inline void right4Block() {
    rightAutoMatchload();
    moveToPoint(-32, -48, false, false, true);
    tongue.setState(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    goalShake(1100);
    autonWingMacro();
}

inline void rightGrab7() {
    chassis.setPose(-46.5, -8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-33, -12, true, false, true, 85);
    moveToPoint(-25, -26, true, false, true, 90);
    tongue.setState(true);
    turnToHeading(-137, true);
    moveToPoint(-43, -47.5, true, false, true, 90);
    turnToHeading(-90, true);
    leftResetPos();
    moveToPoint(-56, -48, true, false, true);
    turnToHeading(-90, true);
    moveDrive(50, 500);
    waitAtMatchload();
}

inline void leftGrab7() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-33, 12, true, false, true, 85);
    moveToPoint(-25, 26, true, false, true, 90);
    tongue.setState(true);
    turnToHeading(-43, true);
    moveToPoint(-43, 47.5, true, false, true, 90);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-56, 48, true, false, true);
    turnToHeading(-90, true);
    moveDrive(50, 600);
    waitAtMatchload();
}


// inline void rightGrab7Diamond() {
//     chassis.setPose(-46.5, -8, 90);
//     intake.setState(IntakeState::INTAKE);
//     moveToPoint(-33, -12, true, false, true, 85);
//     moveToPoint(-25, -26, true, false, true, 90);
//     tongue.setState(true);
//     turnToHeading(-137, true);
//     moveToPoint(-43, -48, true, false, true, 90);
//     turnToHeading(-90, true);
//     leftResetPos();
//     moveToPoint(-56, -49, true, false, true);
//     turnToHeading(-75, true);
//     moveDrive(50, 450);
//     rightMotors.move(100);
//     leftMotors.move(30);
//     pros::delay(300);
//     waitAtMatchload();
// }

inline void rightScoreAndWing() {
    turnToHeading(-90, true);
    moveToPoint(-32, -48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1100);
    autonWingMacroOpposite();
}

inline void right7Block() {
    rightGrab7();
    // rightScoreAndWing();
    turnToHeading(-90, true);
    moveToPoint(-32, -48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1400);
    moveDrive(55, 325);
    wing.setState(true);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(500);
    middleGoal.setState(false);
}

inline void left7Block() {
    leftGrab7();
    turnToHeading(-90, true);
    moveToPoint(-32, 48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1400);
    moveDrive(55, 325);
    wing.setState(true);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(500);
    middleGoal.setState(false);
}

inline void leftLGAndC_CenterFirst() {
    leftGrabBlocks();
    moveToPoint(-15, 17, false, false, false, 90);
    turnToHeading(-45, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
    pros::delay(100);
    middleGoal.setState(true);
    intake.setState(IntakeState::IDLE);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-60, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 350);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-31, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1500);
    autonWingMacro();
}


inline void leftLGAndC_CenterLast() {
    leftGrabBlocks();
    moveToPoint(-24, 27, false, false);
    moveToPoint(-48, 48, false, true);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-60, 48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-31, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(300);
    wing.setState(true);
    intake.setState(IntakeState::OUTTAKE);
    turnToHeading(-105, true);
    resetPos();
    intake.setState(IntakeState::IDLE);
    rightResetPos();
    moveToPoint(-46, 48, true, false);
    moveToPoint(-16, 16, false, true, true, 90);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

// scores 2 on the center goal first, wings the long goal
inline void rightLGAndC_CenterFirst() {
    rightGrabBlocks();
    moveToPoint(-24, -24, false, false);
    intake.setState(IntakeState::IDLE);
    tongue.setState(false);
    moveToPoint(-11, -11, true, true, true);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(900);
    intakeLift.setState(false);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -24, false, false, true);
    tongue.setState(true);
    moveToPoint(-48, -48, false, false);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-60, -48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-31, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1500);
    autonWingMacro();
}

inline void rightLGAndC_CenterLast() {
    rightGrabBlocks();
    moveToPoint(-24, -27, false, false);
    moveToPoint(-48, -48, false, true);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-58, -48, true, false, true);
    moveDrive(50, 170);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-31, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(300);
    wing.setState(true);
    intake.setState(IntakeState::OUTTAKE);
    pros::delay(100);
    intake.setState(IntakeState::IDLE);
    turnToHeading(0, true);
    leftResetPos();
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y + 8, currentPose.theta);
    moveToPoint(-17, -17, true, false, true, 80);
    intake.setState(IntakeState::OUTTAKE);
    moveToPoint(-11, -11, true, false, 90);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(900);
    intakeLift.setState(false);
}

inline void NISAWP() {
    rightLGAndC_CenterLast();
    intake.setState(IntakeState::INTAKE);
    driveDistance(-12);
    intakeLift.setState(false);
    setWings(true);
    moveToPoint(-20, 16, true, true, true, 70);
    tongue.setState(true);
    moveToPoint(-19, 22, true, false, false, 35);
    moveToPoint(-12, 13, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

inline void SAWP13() {
    rightAutoMatchload();
    moveToPoint(-28.5, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
    turnToHeading(0, true);
    leftResetPos();
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y + 6, currentPose.theta);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -26, true, false, true, 90);
    tongue.setState(true);
    chassis.moveToPoint(-24, 20, 1500, {.maxSpeed = 100, .minSpeed = 30, .earlyExitRange = true});
    chassis.waitUntil(16);
    tongue.setState(false);
    chassis.waitUntilDone();
    tongue.setState(true);
    moveToPoint(-50, 48, true, false);
    turnToHeading(-80, true);
    resetPos();
    rightResetPos();
    moveToPoint(-30, 48.5, false, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(800);
    moveToPoint(-50, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-90, true);
    moveDrive(50, 500);
    waitAtMatchload();
    rightResetPos();
    // moveToPoint(-50, 46, false, false, true, 70);
    moveDrive(-60, 330);
    moveDrive(-10, 75);
    moveToPoint(-15, 15, false, true, true, 85);
    pros::delay(200);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 30);
}

inline void leftMatchloadScore() {
    // scores 4 on long goal and clears matchload station
    chassis.setPose(-48, 18, 0);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(0);
    resetPos();
    turnToHeading(-90);
    moveDrive(50, 600);
    waitAtMatchload();
    moveToPoint(-36, 48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2300);
    intake.setState(IntakeState::IDLE);
    driveDistance(10);
    setWings(true);
    moveDrive(-100, 600);
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(-90);
    driveToGoal(true);
    waitAtMatchload();
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void rightMatchloadScore() {

}

// new updated for launch
//inline void skillsAuton() {
    // chassis.setPose(-46.5, 8, 90);
    // intake.setState(IntakeState::INTAKE);
    // moveToPoint(-24, 24, true, false, false, 70);
    // tongue.setState(true);
    // pros::delay(300);
    // moveToPoint(-12, 12, false, true, true, 70);
    // intake.setState(IntakeState::SCOREMID);
    // driveToGoal(false, 40);
    // pros::delay(300);
    // intake.setState(IntakeState::IDLE);
    // moveToPoint(-34, 48, true, false);
    // middleGoal.setState(true);
    // turnToHeading(-90, true);
    // rightResetPos();
    // moveToPoint(-50, 48, true, false, true);
    // turnToHeading(-90, true);
    // intake.setState(IntakeState::INTAKE);
    // moveDrive(65, 500);
    // skillsWaitAtMatchload(); /////////// FIRST MATCHLOAD
    // rightResetPos();
    // moveToPoint(-43, 62, false, false, true);
    // turnToHeading(-90, true);
    // rightResetPos();
    // resetPos();
    // tongue.setState(false);
    // moveToPoint(32, 61, false, false, true, 90);
    // // rightResetPos();
    // turnToHeading(0, true);
    // resetPos();
    // rightResetPos();
    // moveToPoint(36, 48, false, false, false, 80);
    // turnToHeading(90);
    // leftResetPos();
    // moveToPoint(30, 48, false, false, true);
    // driveToGoal(false);
    // tongue.setState(true);
    // intake.setState(IntakeState::SCOREHIGH);
    // pros::delay(2200);
    // leftResetPos();
    // turnToHeading(90);
    // moveToPoint(49, 48, true, false, true);
    // turnToHeading(90, true);
    // intake.setState(IntakeState::INTAKE);
    // moveDrive(60, 500);
    // skillsWaitAtMatchload(); /////////// SECOND MATCHLOAD
    // leftResetPos();
    // moveToPoint(40, 48, false, false, true);
    // moveToPoint(30, 48, false, false, true);
    // driveToGoal(false);
    // tongue.setState(false);
    // intake.setState(IntakeState::SCOREHIGH);
    // pros::delay(1300);
    // intake.setState(IntakeState::SCOREHIGHSLOW);
    // pros::delay(900);
    // moveToPoint(60, 36, true, false, true);
    // intake.setState(IntakeState::INTAKE);
    // turnToHeading(168);
    // moveDrive(60, 550);
    // moveDrive(-2, 200);
    // skillsClearParkZone();
    // moveDrive(60, 500);
    // moveToPoint(48, -24, false, false);
    // turnToHeading(90, true);
    // resetPos();
    // rightResetPos();
    // moveToPoint(19, -17, false, true);
    // turnToHeading(135);
    // driveToGoal(false, 40);
    // tongue.setState(true);
    // intake.setState(IntakeState::SCOREMID);
    // pros::delay(4100);
    // intake.setState(IntakeState::IDLE);
    // middleGoal.setState(true); ////////////////////// SCORED 7
    // moveToPoint(38, -48, true, false);
    // turnToHeading(90, true);
    // rightResetPos();
    // moveToPoint(50, -48, true, false, true);
    // turnToHeading(90, true);
    // intake.setState(IntakeState::INTAKE);
    // moveDrive(65, 500);
    // skillsWaitAtMatchload(); /////// THIRD MATCHLOAD
    // rightResetPos();
    // moveToPoint(43, -62, false, false, true);
    // turnToHeading(90, true);
    // rightResetPos();
    // resetPos();
    // tongue.setState(false);
    // moveToPoint(-32, -61, false, false, true, 90);
    // // rightResetPos();
    // turnToHeading(180, true);
    // resetPos();
    // rightResetPos();
    // moveToPoint(-36,- 48, false, false, false, 80);
    // turnToHeading(-90);
    // leftResetPos();
    // moveToPoint(-30,- 48, false, false, true);
    // driveToGoal(false);
    // tongue.setState(true);
    // intake.setState(IntakeState::SCOREHIGH);
    // pros::delay(2200);
    // leftResetPos();
    // turnToHeading(-90);
    // moveToPoint(-49,-48, true, false, true);
    // turnToHeading(-90, true);
    // intake.setState(IntakeState::INTAKE);
    // moveDrive(60, 500);
    // skillsWaitAtMatchload(); /////// FOURTH MATCHLOAD
    // leftResetPos();
    // moveToPoint(-40, -48, false, false, true);
    // moveToPoint(-30, -48, false, false, true);
    // driveToGoal(false);
    // tongue.setState(false);
    // intake.setState(IntakeState::SCOREHIGH);
    // pros::delay(1300);
    // intake.setState(IntakeState::SCOREHIGHSLOW);
    // pros::delay(800);
    // moveToPoint(-60, -36, true, false, true);
    // intake.setState(IntakeState::INTAKE);
    // turnToHeading(-12);
    // moveDrive(60, 700);
    // moveDrive(-2, 200);
    // driveInParkZone();
//}

inline void skillsAuton(){
    //-----------------------Park zone + Middle Goal 7-----------------------
    chassis.setPose(-55, 0, -90);
    odomLift.setState(true);
    tongue.setState(true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(15, 300);
    tongue.setState(false);
    pros::delay(400);
    moveDrive(70, 400);
    moveDrive(-60, 75);
    moveDrive(60, 75);
    intakeShake(1100);
    moveDrive(-60, 800);
    turnToHeading(-90);
    odomLift.setState(false);
    resetPos();
    // leftResetPos();
    rightResetPos();
    moveToPoint(-10,10, false, true, true);
    turnWithSpeeds(20, -100, -50);
    moveDrive(-60, 150);
    // turnToHeading(-45);
    moveDrive(45, 480);
    // moveToPoint(-18,18, true, false);
    driveToGoal(false, 40);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(2200);
    slowMidGoal = true;
    pros::delay(500);
    //-----------------------1st Matchload -----------------------
    chassis.moveToPoint(-42, 45, 1200, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntil(8);
    intake.setState(IntakeState::INTAKE);
    tongue.setState(true);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-52, 48, true, false, true);
    turnToHeading(-90, true);
    rightResetPos();
    moveDrive(65, 500);
    skillsWaitAtMatchload(); /////////// FIRST MATCHLOAD
    rightResetPos();
	leftMotors.move(-80);
	rightMotors.move(-20);
	pros::delay(200);
    turnToHeading(55, true);
    moveDrive(105, 1100);
    // rightResetPos();
    resetPos();
    leftResetPos();
    moveToPoint(31, 65, true, false, true, 90);
    turnToHeading(35, true);
    turnWithSpeeds(-30, -90, 75);
    //-----------------------1st Long Goal -----------------------
    intake.setState(IntakeState::SCOREHIGH);
    goalShake(2200);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(28, currentPose.y, currentPose.theta);
    turnToHeading(90);
    // leftResetPos();
    moveToPoint(49, 48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    //-----------------------2nd Matchload -----------------------
    skillsWaitAtMatchload();
    // leftResetPos();
    moveToPoint(40, 48, false, false, true);
    moveToPoint(30, 48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(500);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1200);
    // --------------- GOING TO CLEAR PARK ZONE
    moveToPoint(62, 36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(168);
    moveDrive(60, 550);
    moveDrive(-2, 200);
    skillsClearParkZone();
	// tongue.setState(true);
    leftResetPos();
    leftMotors.move(45);
    rightMotors.move(55);
    pros::delay(550);
    leftMotors.move(0);
    rightMotors.move(0);
    moveDrive(-70, 150);
	chassis.turnToHeading(145, 500, {.minSpeed = 80, .earlyExitRange = 20});
    turnToHeading(-165, true);
	leftResetPos();
    resetPos();
	moveToPoint(27, -18, true, true, false, 80);
	moveToPoint(19, 19, true, true, false, 80);
	moveToPoint(12, 12, true, true, true, 80);
     //-----------------------Score 7 Low -----------------------
    intakeLift.setState(true);
    intake.setState(IntakeState::SCORELOW);
	driveToGoal(true);
    pros::delay(3200);
	moveDrive(-60, 250);
	intakeLift.setState(false);
	intake.setState(IntakeState::INTAKE);
    turnToPoint(46, -45, true);
    chassis.moveToPoint(40, -44, 1500, {.minSpeed = 5, .earlyExitRange = 2});
    chassis.waitUntil(16);
    tongue.setState(true);
	turnToHeading(90, true);
	rightResetPos();
	moveToPoint(51, -46.5, true, true, true);
	turnToHeading(90, true);
	rightResetPos();
    moveDrive(65, 500);
	//-----------------------THIRD MATCHLOAD -----------------------
    skillsWaitAtMatchload(); /////////// THIRD MATCHLOAD
    rightResetPos();
	leftMotors.move(-80);
	rightMotors.move(-20);
	pros::delay(200);
    turnToHeading(-120, true);
    moveDrive(105, 1100);
    // rightResetPos();
    resetPos();
    leftResetPos();
    moveToPoint(-31,-65, true, false, true, 90);
    turnToHeading(-145, true);
    turnWithSpeeds(-30, -90, -105);
    //-----------------------4th long goal -----------------------
    intake.setState(IntakeState::SCOREHIGH);
	goalShake(2200);
	// leftResetPos();
    turnToHeading(-90);
    moveToPoint(-50,-48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    skillsWaitAtMatchload(); /////// FOURTH MATCHLOAD
    // leftResetPos();
    // moveToPoint(-40, -48, false, false, true);
    moveToPoint(-30, -48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    goalShake(600);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1200);
    moveToPoint(-53, -35, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-12);
    moveDrive(60, 500);
    moveDrive(-2, 200);
	driveInParkZone();
}

inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
        enableColourSort = false;
    }
    if(startingSide == Selection::LEFT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            left7Block(); // not long control
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            leftLGAndC_CenterLast();
        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            right7Block(); // not long control
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            rightLGAndC_CenterLast();
        }
        if(autonSelectionEnum == Selection::SOLO_AWP) {
            SAWP13();
        }
        if(autonSelectionEnum == Selection::NI_SAWP) {
            NISAWP();
        }
    }
}