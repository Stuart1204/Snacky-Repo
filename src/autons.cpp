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

inline void testingAuto() {
    chassis.setPose(0,0,0);
    moveToPoint(24, 24, true, true);
}

inline void leftSideAuto() {
    chassis.setPose(0,0,0);
}

inline void rightSideAuto() {

}

// grabs blocks under goal
inline void rightGrabBlocks() {
    chassis.setPose(-46.5, -8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -24, true, false, true, 65);
    tongue.setState(true);
    moveToPoint(-18, -38, true, false, true, 60);
    tongue.setState(false);
    moveToPoint(-9, -45, true, false, true, 80);
    tongue.setState(true);
    pros::delay(500);
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
    pros::delay(500);
}

inline void SAWP() {
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::LOWINTAKE);
    //setWings(true);
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(300);
    leftMotors.move(40);
    rightMotors.move(40);
    pros::delay(300);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(600); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    driveDistance(7);
    turnToHeading(-90);
    leftResetPos();
    //resetPos();
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-27, -27, true, true, false, 70);
    //moveToPoint(-12, -14, true, true, false, 35);
    moveToPoint(-12, -12, true, false, false, 50);
    turnToHeading(45);
    intakeLift.setState(true);
    intake.setState(IntakeState::SCORELOW);
    driveToGoal(true);
    pros::delay(500);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-12);
    intakeLift.setState(false);
    setWings(true);
    moveToPoint(-22, 15, true, true, true, 70);
    moveToPoint(-22, 22, true, false, false, 35);
    turnToHeading(-45);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

inline void NISIGSAWP() {
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(55, 550);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    turnToHeading(-90);
    leftResetPos();
    pros::delay(1000); // scoring all blocks
    driveDistance(7);
    //resetPos();
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-27, -27, true, true, true, 70);
    tongue.setState(true);
    moveToPoint(-18, -18, true, false, true, 70);
    tongue.setState(false);
    moveToPoint(-12, -12, true, false, false, 50);
    turnToHeading(45);
    intakeLift.setState(true);
    intake.setState(IntakeState::SCORELOW);
    driveToGoal(true);
    pros::delay(650);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-12);
    intakeLift.setState(false);
    setWings(true);
    moveToPoint(-22, 16, true, true, true, 70);
    tongue.setState(true);
    moveToPoint(-22, 22, true, false, false, 35);
    turnToHeading(-45);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
    // pros::delay(1600);
    // moveToPoint(-60, 27, true, false, false, 127);
}

inline void NISIGSAWP2() {
    rightGrabBlocks();
    moveToPoint(-24, -24, false, false);
    intake.setState(IntakeState::IDLE);
    tongue.setState(false);
    moveToPoint(-10, -10, true, true, true);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(1200);
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
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1500);
    turnWithSpeeds(127, -50, -20);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 18, true, false, true, 70);
    tongue.setState(true);
    moveToPoint(-23, -23, true, false, true);
    turnToHeading(-20);
    resetPos();
    moveToPoint(-14, 14, false, false, true, 75);
    intake.setState(IntakeState::SCOREMID);
    turnToHeading(-45);
    driveToGoal(false);
}

inline void SIGSAWP() { // not good for g1
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(70, 500);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false, true);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    pros::delay(900); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    turnWithSpeeds(127, -50, -20);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-26, -24, true, false, true);
    turnToHeading(0);
    leftResetPos();
    moveToPoint(-24, 17, true, false, true, 70);
    moveToPoint(-9, 41, true, false, true, 70);
    tongue.setState(true);
    pros::delay(300);
    moveToPoint(-12, 11, false, false, true, 75);
    intake.setState(IntakeState::SCOREMID);
    turnToHeading(-45);
    driveToGoal(false);
    pros::delay(100);
    stopScoringMid();
    moveToPoint(-43, 45, true, false, true);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-26, 48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    moveToPoint(-23, 48, false, false, true);
}

inline void SIGSAWP2() { // gets 2 matchloads // hog all matchloads and field
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(70, 500);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false, true);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    pros::delay(900); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    turnWithSpeeds(127, -50, -20);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-26, -24, true, false, true);
    turnToHeading(0);
    leftResetPos();
    moveToPoint(-24, 18, true, false, true, 70);
    tongue.setState(true);
    pros::delay(300);
    turnToHeading(-20);
    resetPos();
    moveToPoint(-16, 16, false, false, true, 75);
    intake.setState(IntakeState::SCOREMID);
    turnToHeading(-45);
    driveToGoal(false);
    pros::delay(300);
    stopScoringMid();
    moveToPoint(-43, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-56, 48, true, false, true);
    turnToHeading(-90);
    moveDrive(70, 500);
    waitAtMatchload();
    moveToPoint(-31, 48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    moveToPoint(-26, 48, false, false, true);
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
    moveToPoint(-32, -48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1800); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    driveDistance(9);
    turnToHeading(-90);
    leftResetPos();
    //resetPos();
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
    moveToPoint(-17, 17, true, true, true);
    turnToHeading(-45);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

// untested
inline void rightLongGoalControlAuton() {
    // for the right side
    rightGrabBlocks();
    moveToPoint(-24, -24, false, false);
    tongue.setState(false);
    moveToPoint(-48, -48, false, true);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-60, -48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2500);
    driveDistance(10);
    intake.setState(IntakeState::IDLE);
    setWings(true);
    moveDrive(-70, 600);
}

// untested
inline void leftLongGoalControlAuton() {
    // for the left side
    leftGrabBlocks();
    moveToPoint(-24, 24, false, false);
    tongue.setState(false);
    moveToPoint(-48, 48, true, true);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-60, 48, true, false, true);
    moveDrive(50, 350);
    waitAtMatchload();
    moveToPoint(-36, 48, false, false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2500);
    driveDistance(10);
    intake.setState(IntakeState::IDLE);
    setWings(true);
    moveDrive(-70, 600);
}

// new v2, gets the blocks under long goal, scores 1-2 on high middle, scores rest on long goal
inline void leftLongGoalAndCenterAuton() {
    leftGrabBlocks();
    moveToPoint(-24, 24, false, false);
    moveToPoint(-15, 15, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
    pros::delay(400);
    stopScoringMid();
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

// new v2, gets the blocks under long goal, scores 1-2 on low middle, scores rest on long goal
inline void rightLongGoalAndCenterAuton() {
    rightGrabBlocks();
    moveToPoint(-24, -24, false, false);
    intake.setState(IntakeState::IDLE);
    tongue.setState(false);
    moveToPoint(-10, -10, true, true, true);
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
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2000);
    driveDistance(10);
    intake.setState(IntakeState::IDLE);
    setWings(true);
    moveDrive(-70, 600);
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
    chassis.setPose(-63, -14, 0);
    intake.setState(IntakeState::INTAKE);
    odomLift.setState(true);
    skillsClearParkZone();
    odomLift.setState(false);
    tongue.setState(true);
    turnToHeading(15);
    leftResetPos();
    resetPos();
    moveToPoint(-48, 48, true, false, true);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-37, 48, false, false, true);
    moveToPoint(-29, 48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(true);
    pros::delay(1100);
    rightResetPos();
    moveToPoint(-57, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-90);
    moveDrive(80, 500);
    skillsWaitAtMatchload();
    rightResetPos();
    moveToPoint(-37, 48, false, false, true);
    moveToPoint(-29, 48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(false);
    pros::delay(1900);
    driveDistance(10);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 25, true, true, true);
    moveToPoint(24, 24, true, true, true);
    tongue.setState(true);
    turnToHeading(90);
    resetPos();
    pros::delay(200);
    turnToPoint(9, 9, true);
    tongue.setState(false);
    moveToPoint(14, 14, true, false, true);
    intakeLift.setState(true);
    moveToPoint(9, 9, true, false, true);
    intake.setState(IntakeState::OUTTAKE);
    pros::delay(1100);
    intakeLift.setState(false);
    moveToPoint(36, 36, false, false, true);
    tongue.setState(true);
    moveToPoint(48, 48, false, false, true);
    intake.setState(IntakeState::IDLE);
    turnToHeading(90);
    leftResetPos();
    moveToPoint(57, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(90);
    moveDrive(80, 500);
    skillsWaitAtMatchload();
    leftResetPos();
    moveToPoint(37, 48, false, false, true);
    moveToPoint(29, 48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(false);
    pros::delay(1000);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1000);
    moveToPoint(60, 36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(168);
    moveDrive(60, 550);
    moveDrive(-2, 200);
    skillsClearParkZone();
    tongue.setState(true);
    turnToHeading(-165);
    leftResetPos();
    resetPos();
    moveToPoint(48, -48, true, false, true);
    turnToHeading(90);
    rightResetPos();
    moveToPoint(37, -48, false, false, true);
    moveToPoint(29, -48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(true);
    pros::delay(1200);
    rightResetPos();
    moveToPoint(57, -48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(90);
    moveDrive(80, 500);
    skillsWaitAtMatchload();
    rightResetPos();
    moveToPoint(37, -48, false, false, true);
    moveToPoint(29, -48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(false);
    pros::delay(1900);
    driveDistance(10);
    intake.setState(IntakeState::SCOREHIGH);
    moveToPoint(30, -30, true, true, true);
    // tongue.setState(true);
    // moveToPoint(20, -20, true, false, true, 80);
    // turnToPoint(9, -9, false);
    // intake.setState(IntakeState::SCOREMID);
    // driveToGoal(false);
    // pros::delay(1000);
    // moveToPoint(11, -26, true, false,true);
    moveToPoint(-26, -31, true, false, true);
    resetPos();
    tongue.setState(true);
    moveToPoint(-48, -48, true, false, true);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-57, -48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-90);
    moveDrive(80, 500);
    skillsWaitAtMatchload();
    leftResetPos();
    moveToPoint(-37, -48, false, false, true);
    moveToPoint(-29, -48, false, false, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    tongue.setState(false);
    pros::delay(1000);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(700);
    moveToPoint(-60,-36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-15);
    driveInParkZone();
}

inline void elims7BallLeftSide() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-26, 26, true, false, true, 100);
    tongue.setState(true);
    pros::delay(100);
    moveToPoint(-36, 40, true, false, true, 100);
    moveToPoint(-45, 48, true, false, true, 100);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-60, 48, true, false, true);
    rightResetPos();
    moveDrive(60, 350);
    waitAtMatchload(750);
    moveToPoint(-30, 48, false, false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2250);
}
inline void elims7BallRightSide() {
    chassis.setPose(-46.5, -8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-26, -26, true, false, true, 85);
    tongue.setState(true);
    pros::delay(100);
    moveToPoint(-34, -40, true, false, true, 100);
    moveToPoint(-48, -49, true, false, true, 100);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-60, -48, true, false, true);
    leftResetPos();
    moveDrive(60, 350);
    waitAtMatchload(700);
    moveToPoint(-30, -48, false, false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    enableColourSort = false;
    // intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1500);
    int notColour;
    if(allianceColour == AllianceColour::BLUE) {
        notColour = AllianceColour::RED;
    }
    if(allianceColour == AllianceColour::RED) {
        notColour = AllianceColour::BLUE;
    }
    uint32_t startTime = pros::millis();
    while(true) {
        if(checkColour(colourSortSensor) == notColour) {
            break;
        }
        if (pros::millis() - startTime > 1500) {
            break;
        }
        pros::delay(20);
    }
    setWings(true);
    moveDrive(50, 350);
    moveDrive(-70, 350);
}

// inline void wingMacro() {
//     chassis.turnToHeading(-45, 500, {.minSpeed = 90});
//     driveDistance(4, 70);
//     chassis
// }
inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
        enableColourSort = false;
    }
    if(startingSide == Selection::LEFT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            leftLongGoalControlAuton();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            leftLongGoalAndCenterAuton();
        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            // rightLongGoalControlAuton();
            elims7BallRightSide();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            rightLongGoalAndCenterAuton();
        }
        if(autonSelectionEnum == Selection::SOLO_AWP) {
            //SIGSAWP();
            NISIGSAWP();
        }
        if(autonSelectionEnum == Selection::NI_SAWP) {
            NISIGSAWP();
        }
    }
}