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
    leftMotors.move(5);
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
    moveToPoint(-18, 37, true, false, true, 60);
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
    moveDrive(65, 500);
    waitAtMatchload();
    leftResetPos();
}

inline void right4Block() {
    rightAutoMatchload();
    moveToPoint(-32, -48, false, false, true);
    tongue.setState(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    moveDrive(-50, 1200);
    startAutoWing();
    while(enableAutoWing) {
        autoWing();
    }
    leftMotors.move(-70);
    rightMotors.move(-50);
    pros::delay(900);
    leftMotors.move(-40);
    rightMotors.move(-40);
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

inline void rightGrab7Diamond() {
    chassis.setPose(-46.5, -8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-33, -12, true, false, true, 85);
    moveToPoint(-25, -26, true, false, true, 90);
    tongue.setState(true);
    turnToHeading(-137, true);
    moveToPoint(-43, -48, true, false, true, 90);
    turnToHeading(-90, true);
    leftResetPos();
    moveToPoint(-56, -49, true, false, true);
    turnToHeading(-75, true);
    moveDrive(50, 450);
    rightMotors.move(100);
    leftMotors.move(30);
    pros::delay(300);
    waitAtMatchload();
}

inline void rightScoreAndWing() {
    moveToPoint(-32, -48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1100);
    autonWingMacroOpposite();
}

inline void right7Block() {
    rightGrab7();
    rightScoreAndWing();
}

inline void right7BlockDiamond() {
    rightGrab7Diamond();
    rightScoreAndWing();
}

inline void left7Block() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-33, 12, true, false, true, 85);
    moveToPoint(-25, 26, true, false, true, 90);
    tongue.setState(true);
    turnToHeading(-42, true);
    moveToPoint(-43, 48, true, false, true, 90);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-56, 48, true, false, true);
    turnToHeading(-90, true);
    moveDrive(60, 600);
    waitAtMatchload();
    moveToPoint(-32, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1100);
    autonWingMacro();
}

// new v2, gets the blocks under long goal, scores 1-2 on high middle, scores rest on long goal
inline void leftLongGoalAndCenterAuton() {
    leftGrabBlocks();
    moveToPoint(-24, 24, false, false);
    moveToPoint(-15, 15, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
    pros::delay(400);
    // stopScoringMid();
    moveToPoint(-28, 28, true, false, true);
    intake.setState(IntakeState::INTAKE);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-60, 48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    moveToPoint(-36, 48, false, false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2000);
    driveDistance(10);
    intake.setState(IntakeState::IDLE);
    setWings(true);
    moveDrive(-70, 600);
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
    moveToPoint(-60, -48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-31, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
    wing.setState(true);
    pros::delay(200);
    turnToHeading(0, true);
    leftResetPos();
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y + 6, currentPose.theta);
    moveToPoint(-18, -18, true, false, true, 80);
    moveToPoint(-11, -11, true, false, 90);
}

inline void SAWP13() {
    rightAutoMatchload();
    moveToPoint(-31, -48, false, false, true);
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
    turnToHeading(-75, true);
    resetPos();
    rightResetPos();
    moveToPoint(-32, 48.5, false, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(800);
    moveToPoint(-50, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 500);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-50, 46, false, false, true, 70);
    moveToPoint(-15, 15, false, true, true);
    pros::delay(200);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

inline void NISAWP() {
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    //setWings(true);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(300);
    leftMotors.move(40);
    rightMotors.move(40);
    pros::delay(300);
    waitAtMatchload();
    moveToPoint(-32, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    pros::delay(1200);
    intake.setState(IntakeState::IDLE);
    turnToHeading(0, true);
    leftResetPos();
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-27, -27, true, true, false, 50);
    //moveToPoint(-12, -14, true, true, false, 35);
    moveToPoint(-12, -12, true, false, false, 35);
    turnToHeading(45);
    intakeLift.setState(true);
    intake.setState(IntakeState::LOWOUTTAKE);
    driveToGoal(true);
    pros::delay(500);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-20, -16, false, false, true);
    intakeLift.setState(false);
    setWings(true);
    moveToPoint(-19, 19, true, true, true);
    turnToHeading(-45);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
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
inline void skillsAuton() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 24, true, false, false, 70);
    tongue.setState(true);
    pros::delay(300);
    moveToPoint(-12, 12, false, true, true, 70);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 40);
    pros::delay(300);
    intake.setState(IntakeState::IDLE);
    middleGoal.setState(true);
    moveToPoint(-34, 48, true, false);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-52, 48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    skillsWaitAtMatchload(); /////////// FIRST MATCHLOAD
    rightResetPos();
    moveToPoint(-43, 62, false, false, true);
    turnToHeading(-90, true);
    rightResetPos();
    resetPos();
    tongue.setState(false);
    moveToPoint(32, 61, false, false, true, 90);
    // rightResetPos();
    turnToHeading(0, true);
    resetPos();
    rightResetPos();
    moveToPoint(36, 48, false, false, false, 80);
    turnToHeading(90);
    leftResetPos();
    moveToPoint(30, 48, false, false, true);
    driveToGoal(false);
    tongue.setState(true);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2200);
    leftResetPos();
    turnToHeading(90);
    moveToPoint(50, 48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 500);
    skillsWaitAtMatchload(); /////////// SECOND MATCHLOAD
    leftResetPos();
    moveToPoint(40, 48, false, false, true);
    moveToPoint(30, 48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1300);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(900);
    moveToPoint(60, 36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(168);
    moveDrive(60, 550);
    moveDrive(-2, 200);
    skillsClearParkZone();
    moveDrive(60, 500);
    moveToPoint(48, -24, false, false);
    turnToHeading(90, true);
    resetPos();
    rightResetPos();
    moveToPoint(19, -17, false, true);
    turnToHeading(135);
    driveToGoal(false, 40);
    tongue.setState(true);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(3000);
    intake.setState(IntakeState::IDLE);
    middleGoal.setState(true); ////////////////////// SCORED 7
    moveToPoint(38, -48, true, false);
    turnToHeading(90, true);
    rightResetPos();
    moveToPoint(52, -48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    skillsWaitAtMatchload(); /////// THIRD MATCHLOAD
    rightResetPos();
    moveToPoint(43, -62, false, false, true);
    turnToHeading(90, true);
    rightResetPos();
    resetPos();
    tongue.setState(false);
    moveToPoint(-32, -61, false, false, true, 90);
    // rightResetPos();
    turnToHeading(180, true);
    resetPos();
    rightResetPos();
    moveToPoint(-36,- 48, false, false, false, 80);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-30,- 48, false, false, true);
    driveToGoal(false);
    tongue.setState(true);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2200);
    leftResetPos();
    turnToHeading(-90);
    moveToPoint(-50,-48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 500);
    skillsWaitAtMatchload(); /////// FOURTH MATCHLOAD
    leftResetPos();
    moveToPoint(-40, -48, false, false, true);
    moveToPoint(-30, -48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1300);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(800);
    moveToPoint(-60, -36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-12);
    moveDrive(60, 550);
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
            // leftLongGoalControlAuton();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            leftLongGoalAndCenterAuton();
        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            // rightLongGoalControlAuton();
            // elims7BallRightSide();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            rightLGAndC_CenterFirst();
        }
        if(autonSelectionEnum == Selection::SOLO_AWP) {
            //SIGSAWP();
            // NISIGSAWP();
        }
        if(autonSelectionEnum == Selection::NI_SAWP) {
            // NISIGSAWP();
        }
    }
}