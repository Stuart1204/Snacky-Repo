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
    moveToPoint(-36, -48.5, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(600); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    driveDistance(7);
    turnToHeading(-86);
    leftResetPos();
    resetPos();
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-27, -27, true, true, false, 70);
    //moveToPoint(-12, -14, true, true, false, 35);
    moveToPoint(-12, -12, true, false, false, 40);
    turnToHeading(45);
    intakeLift.setState(true);
    driveToGoal(true);
    intake.setState(IntakeState::SCORELOW);
    pros::delay(800);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-10);
    intakeLift.setState(false);
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
    enableColourSort = false;
    chassis.setPose(-48, 18, 0);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
    // turnToHeading(0);
    // resetPos();
    turnToHeading(-90);
    setWings(true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload();
    driveDistance(-1);
    turnToHeading(-90);
    rightResetPos();
    moveToPoint(-34, 36, false, false, true);
    turnToHeading(-90);
    rightResetPos();
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    moveToPoint(37, 36, false, false);
    turnToHeading(-135);
    intake.setState(IntakeState::INTAKE);
    driveDistance(9);
    moveToPoint(38, 43, false, false);
    turnToHeading(0);
    driveToGoalDistance();
    turnToHeading(90);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false); // scoring on long goal
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    pros::delay(2800);
    moveToPoint(57, 47, true, false, true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    turnToHeading(90);
    resetPos();
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload(); // waiting at matchload #2
    tongue.setState(false);
    driveDistance(-1);
    turnToHeading(90);
    leftResetPos();
    moveToPoint(38, 48, false, false, true);
    // turnToHeading(0);
    // driveToGoalDistance();
    intake.setState(IntakeState::IDLE);
    setWings(false);
    turnToHeading(90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3200);
    intake.setState(IntakeState::IDLE);
    driveDistance(9);
    turnToPoint(43, -36, true);
    tongue.setState(true);
    moveToPoint(43, -36, true, false);
    turnToHeading(180);
    resetPos();
    resetPos();
    leftResetPos();
    leftResetPos();
    moveToPoint(47, -47, true, true);
    turnToHeading(90);
    // going to clear matchload #3

    setWings(true);
    intake.setState(IntakeState::INTAKE);
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads #3
    pros::delay(800);
    skillsWaitAtMatchload();
    turnToHeading(90);
    rightResetPos();
    moveToPoint(34, -36, false, false, true);
    turnToHeading(90);
    rightResetPos();
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    moveToPoint(-37, -36, false, false);
    turnToHeading(45);
    intake.setState(IntakeState::INTAKE);
    driveDistance(9);
    moveToPoint(-38,-43, false, false);
    turnToHeading(180);
    driveToGoalDistance();
    turnToHeading(-90);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false); // scoring on long goal
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    pros::delay(2800);
    moveToPoint(-57,-47, true, false, true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    turnToHeading(-90);
    //resetPos();
    leftMotors.move(50);
    rightMotors.move(50); // collecting matchloads
    pros::delay(800);
    skillsWaitAtMatchload(); // waiting at matchload #4
    tongue.setState(false);
    driveDistance(-1);
    turnToHeading(-90);
    leftResetPos();
    moveToPoint(-38,-48, false, false, true);
    // turnToHeading(180);
    // driveToGoalDistance();
    intake.setState(IntakeState::IDLE);
    setWings(false);
    turnToHeading(-90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3200);
    intake.setState(IntakeState::IDLE);
    driveDistance(9);
    moveToPoint(-66, -17, true, true, true);
    tongue.setState(true);
    chassis.swingToHeading(-5, lemlib::DriveSide::RIGHT, 1000, {.minSpeed = 90});
	chassis.waitUntilDone();
    odomLift.setState(true);
   	moveDrive(90, 800);
	moveDrive(0, 100);
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