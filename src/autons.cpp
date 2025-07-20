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

inline void SAWP() {
    chassis.setPose(-48, -17.5, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::LOWINTAKE);
    //setWings(true);
    leftMotors.move(70);
    rightMotors.move(70);
    pros::delay(300);
    leftMotors.move(50);
    rightMotors.move(50);
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
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-27, -27, true, true, false, 70);
    //moveToPoint(-12, -14, true, true, false, 35);
    turnToHeading(45);
    driveDistance(23, 35);
    intake.setState(IntakeState::OUTTAKE);
    pros::delay(800);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-10);
    setWings(true);
    moveToPoint(-20, 15, true, true, true, 70);
    moveToPoint(-22, 22, true, false, false, 35);
    turnToHeading(-45);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

inline void rightLongGoalControlAuton() {
    // for the right side
    chassis.setPose(-48, -17.5, 180);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    moveToPoint(-36, -22, true, true, true, 100);
    moveToPoint(-22, -26, true, false, false, 25);
    tongue.setState(true);
    moveToPoint(-50, -47, false, true, false, 90);
    turnToHeading(-90);
    leftResetPos();
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(600);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2750);
    intake.setState(IntakeState::IDLE);
    driveDistance(8);
    turnToHeading(-90);
    leftResetPos();
    setWings(true);
    moveToPoint(-36, -36, true, false, true);
    moveToPoint(-29, -40, false, true, true, 80);
    moveToPoint(-13, -45, false, false, true, 60);
    setWings(false);
    pros::delay(500);
    leftMotors.move(-70);
}

inline void leftLongGoalControlAuton() {
    // for the left side
    chassis.setPose(-48, 17.5, 0);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    moveToPoint(-36, 22, true, true, true, 100);
    moveToPoint(-22, 26, true, false, false, 25);
    tongue.setState(true);
    moveToPoint(-50, 48.5, false, true, false, 90);
    turnToHeading(90);
    leftResetPos();
    turnToHeading(-90);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(600);
    waitAtMatchload();
    moveToPoint(-36, 48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2750);
    intake.setState(IntakeState::IDLE);
    driveDistance(8);
    setWings(true);
    moveToPoint(-36, 36, true, false, true);
    moveToPoint(-29, 40, false, true, true, 80);
    moveToPoint(-13, 45, false, false, true, 60);
    setWings(false);
    pros::delay(500);
    rightMotors.move(-70);
}

inline void leftLongGoalAndCenterAuton() {
    // scores 4 on the long goal and 3 on the center goal. does not worry about control bonus
    chassis.setPose(-48, 17.5, 0);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(0);
    resetPos();
    turnToHeading(-90);
    moveDrive(55, 600);
    waitAtMatchload();
    moveToPoint(-36, 48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2000);
    intake.setState(IntakeState::IDLE);
    driveDistance(8);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    moveToPoint(-39, 26, true, true);
    moveToPoint(-18, 22, true, true, false, 35);
    driveDistance(-8);
    //moveToPoint(-19, 15, false, false, true);
    turnToHeading(-38);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREMID);
}

inline void rightLongGoalAndCenterAuton() {
    // scores 4 on the long goal and 3 on the center goal. does not worry about control bonus
    chassis.setPose(-48, -17.5, 180);
    intake.setState(IntakeState::INTAKEWITHEJECT);
    setWings(true);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    turnToHeading(-90);
    leftResetPos();
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(550);
    waitAtMatchload();
    moveToPoint(-36, -48, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(2000);
    intake.setState(IntakeState::IDLE);
    driveDistance(10);
    turnToHeading(-90);
    leftResetPos();
    intake.setState(IntakeState::LOWINTAKE);
    moveToPoint(-36, -23, true, true, false, 70);
    turnToHeading(0);
    leftResetPos();
    setWings(true);
    moveToPoint(-23, -23, true, true, false, 50);
    //moveToPoint(-12, -14, true, true, false, 35);
    turnToHeading(45);
    driveDistance(20, 35);
    intake.setState(IntakeState::OUTTAKE);
}

inline void skillsAuton() {
    chassis.setPose(-48, 17.5, 0);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    turnToHeading(0);
    resetPos();
    turnToHeading(-90);
    setWings(true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload();
    driveDistance(-8);
    moveToPoint(-30, 59, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    moveToPoint(43, 58, false, true);
    turnToHeading(0);
    resetPos(); // resetting position with top wall
    moveToPoint(43, 48.5, false, false);
    turnToHeading(90);
    setWings(false);
    driveToGoal(false); // scoring on long goal
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    pros::delay(2200);
    moveToPoint(57, 47, true, true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    turnToHeading(90);
    //resetPos();
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload(); // waiting at matchload #2
    tongue.setState(false);
    driveDistance(-10);
    turnToHeading(90);
    leftResetPos();
    moveToPoint(24, 24, true, true, false, 70);
    moveToPoint(43, 48, false, false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    turnToHeading(90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3200);
    intake.setState(IntakeState::IDLE);
    driveDistance(10);
    turnToPoint(47, -39, true);
    tongue.setState(true);
    moveToPoint(47, -39, true, false);
    turnToHeading(180);
    resetPos();
    resetPos();
    leftResetPos();
    leftResetPos();
    moveToPoint(47, -48, true, true);
    turnToHeading(90);
    // going to clear matchload #3

    setWings(true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads #3
    pros::delay(800);
    skillsWaitAtMatchload();
    driveDistance(-8);
    moveToPoint(30, -59, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    moveToPoint(-43, -58, false, true);
    turnToHeading(180);
    resetPos(); // resetting position with top wall
    moveToPoint(-43,-48.5, false, false);
    turnToHeading(-90);
    setWings(false);
    driveToGoal(false); // scoring on long goal
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    pros::delay(2200);
    moveToPoint(-57, -47, true, true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    turnToHeading(-90);
    //resetPos();
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload(); // waiting at matchload #4
    tongue.setState(false);
    driveDistance(-10);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-24, -24, true, true, false, 70);
    moveToPoint(-43, -48.5, false, false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3200);
    driveDistance(10);
    moveToPoint(-65, -16, true, true);
    tongue.setState(true);
    turnToHeading(-20);
    odomLift.setState(true);
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
            leftLongGoalControlAuton();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            leftLongGoalAndCenterAuton();
        }
        if(autonSelectionEnum == Selection::NOTHING) {
            return;
        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            rightLongGoalControlAuton();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            rightLongGoalAndCenterAuton();
        }
        if(autonSelectionEnum == Selection::SOLO_AWP) {
            SAWP();
        }
        if(autonSelectionEnum == Selection::NOTHING) {
            return;
        }
    }
}