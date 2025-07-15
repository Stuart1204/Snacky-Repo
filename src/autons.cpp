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

inline void oldSAWP() {
    chassis.setPose(-48, 24 - 6.75 - .5 ,90);
    setWings(true);
    intake.setState(IntakeState::IDLE);
    moveToPoint(-35, 21, true, false, true);
    intake.setState(IntakeState::INTAKE);
    driveDistance(15, 30); // picking up the three clump of balls
    moveToPoint(-37, 48, false, true);
    turnToHeading(-90);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false); // driving to the long goal
    intake.setState(IntakeState::SCOREHIGH); // scoring balls onto long goal
    tongue.setState(true);
    pros::delay(1200);  
    moveToPoint(-57, 49, true, true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    leftMotors.move(80);
    rightMotors.move(80); // collecting matchloads
    pros::delay(500);
    waitAtMatchload();
    intake.setState(IntakeState::IDLE);
    //driveDistance(-10);
    moveToPoint(-20, 20, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
    pros::delay(1200);
    moveToPoint(-20, -0, true, false, true);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -20, true, true);
    turnToPoint(-12,-12, true);
    intake.setState(IntakeState::OUTTAKE);
    moveToPoint(-12, -12, true, false);
}

inline void SAWP() {
    chassis.setPose(-48, -17, 180);
    tongue.setState(true);
    moveToPoint(-48, -48, true, false);
    //resetPos();
    turnToHeading(-90);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    leftMotors.move(70);
    rightMotors.move(70);
    pros::delay(300);
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(300);
    waitAtMatchload();
    moveToPoint(-36, -49, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(1000); // 2000 if wanted to score all balls
    intake.setState(IntakeState::IDLE);
    driveDistance(7);
    intake.setState(IntakeState::LOWINTAKE);
    setWings(true);
    moveToPoint(-30, -31, true, true, false, 70);
    moveToPoint(-12, -12, true, true, false, 35);
    chassis.turnToHeading(47, 500);
    intake.setState(IntakeState::OUTTAKE);
    pros::delay(1200);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-6);
    setWings(true);
    moveToPoint(-23, 21, true, true, false, 70);
    turnToPoint(0, 0, false);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false);
}

inline void longGoalControlAuton() {
    // for the right side
    chassis.setPose(-48, -17, 180);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    moveToPoint(-36, -22, true, true, true, 100);
    moveToPoint(-22, -26, true, false, false, 35);
    tongue.setState(true);
    moveToPoint(-50, -47, false, true);
    turnToHeading(-90);
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(600);
    waitAtMatchload();
    moveToPoint(-36, -49, false, false);
    turnToHeading(-90);
    tongue.setState(false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3000);
    intake.setState(IntakeState::IDLE);
    driveDistance(8);
    setWings(true);
    moveToPoint(-36, -36, true, false, true);
    moveToPoint(-29, -40, false, true, true, 80);
    moveToPoint(-12, -45, false, false, true, 60);
    setWings(false);
    pros::delay(500);
    leftMotors.move(-100);
}

inline void skillsAuton() {
    chassis.setPose(-48, 17, 0);
    tongue.setState(true);
    moveToPoint(-48, 48, true, false);
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
    moveToPoint(43, 48, false, false);
    turnToHeading(90);
    setWings(false);
    driveToGoal(false); // scoring on long goal
    intake.setState(IntakeState::SCOREHIGH);
    tongue.setState(true);
    pros::delay(2500);
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
    moveToPoint(24, 24, true, true, false, 70);
    moveToPoint(43, 47, false, false);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    turnToHeading(90);
    driveToGoal(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(3750);
    intake.setState(IntakeState::IDLE);
    driveDistance(10);
    turnToPoint(47, -48, true);
    tongue.setState(true);
    moveToPoint(47, -48, true, false);
    turnToHeading(180);
    resetPos();
    turnToHeading(90);
}

