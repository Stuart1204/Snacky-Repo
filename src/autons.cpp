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

inline void rightAutoMatchload() {
    chassis.setPose(-48, -17.5, 180);
    resetPos();
    tongue.setState(true);
    moveToPoint(-48, -48, true, false, false);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(70, 500);
    waitAtMatchload();
    leftResetPos();
}

inline void rightFourBlock() {
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
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 500);
    skillsWaitAtMatchload(); /////////// SECOND MATCHLOAD
    leftResetPos();
    moveToPoint(40, 48, false, false, true);
    moveToPoint(30, 48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1200);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1000);
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
    moveToPoint(18, -18, false, true);
    turnToHeading(135);
    driveToGoal(false, 40);
    tongue.setState(true);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(3000);
    intake.setState(IntakeState::IDLE);
    middleGoal.setState(true); ////////////////////// SCORED 7
    moveToPoint(34, -48, true, false);
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
    intake.setState(IntakeState::INTAKE);
    moveDrive(50, 500);
    skillsWaitAtMatchload(); /////// FOURTH MATCHLOAD
    leftResetPos();
    moveToPoint(-40, -48, false, false, true);
    moveToPoint(-30, -48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1200);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1000);
    moveToPoint(-60, -36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-12);
    moveDrive(60, 550);
    moveDrive(-2, 200);
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
            // leftLongGoalControlAuton();
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
            // NISIGSAWP();
        }
        if(autonSelectionEnum == Selection::NI_SAWP) {
            // NISIGSAWP();
        }
    }
}