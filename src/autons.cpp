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
	turnToHeading(120, true);
    moveDrive(50, 100);
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
    chassis.setPose(-48, -16, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, -24, true, false, true, 100);
    moveToPoint(-12.5, -44, true, false, true, 90);
    tongue.setState(true);
    pros::delay(220);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x+4, currentPose.y+4, currentPose.theta);
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
    // middleGoal.setState(false);
}

inline void right7BlockWing() {
    rightGrab7();
    // rightScoreAndWing();
    turnToHeading(-90, true);
    moveToPoint(-32, -48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1400);
    autonWingMacro();
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

inline void left7BlockWing() {
    leftGrab7();
    turnToHeading(-90, true);
    moveToPoint(-32, 48, false, false, true, 90);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1400);
    autonWingMacro();
}

inline void rightLGAndCRush() { // 1010g auto
    chassis.setPose(-48, -18, 0);
    moveToPoint(-48, -46, false, false, true, 90);
    tongue.setState(true);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 435);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-28.5, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
    turnToHeading(0, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y + 6, currentPose.theta);
    moveToPoint(-24, -26, true, false, true, 90);
    // tongue.setState(true);
    moveToPoint(-17, -17, true, false, true, 50);
    intake.setState(IntakeState::UPPEROUTTAKE);
    // tongue.setState(false);
    moveToPoint(-12, -11, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(750);
    // pros::delay(500); // extra delay
    intakeLift.setState(false);
    chassis.moveToPoint(-19, -29, 1000, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.setState(IntakeState::IDLE);
    turnToHeading(90, true);
    wing.setState(false);
    currentPose = chassis.getPose();
    moveToPoint(-9, currentPose.y, true, false, true, 50);
    leftMotors.move(-30);
    rightMotors.move(30);
    intakeLift.setState(false);
}

inline void leftLGAndCRush() { //1010g auto
    chassis.setPose(-48, 18, 180);
    moveToPoint(-48, 44, false, false, true, 90);
    tongue.setState(true);
    turnToHeading(-90, true);
    rightResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-28.5, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
    turnToHeading(180, true);
    rightResetPos();
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y - 3, currentPose.theta);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 24, true, false, true, 90);
    tongue.setState(true);
    moveToPoint(-12, 12, false, true, true, 60);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(750);
    chassis.moveToPoint(-25, 31, 1000, {.minSpeed = 70, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.setState(IntakeState::IDLE);
    turnToHeading(-110, true);
    wing.setState(false);
    currentPose = chassis.getPose();
    moveToPoint(-16, currentPose.y, false, false, true, 50);
    rightMotors.move(-35);
    leftMotors.move(3);
}

inline void rightLGAndC() {
    // setup to get blocks under the long goal
    rightGrabBlocks();
    intake.setState(IntakeState::LOWINTAKEUPOUTTAKE);
    moveToPoint(-24, -26, false, false, true);
    tongue.setState(false);
    turnToHeading(45, true);
    intake.setState(IntakeState::UPPEROUTTAKE);
    moveToPoint(-13, -13, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(600);
    intake.setState(IntakeState::INTAKE);
    intakeLift.setState(false);
    chassis.moveToPoint(-43, -51, 1500, {.forwards = false, .minSpeed = 10, .earlyExitRange = 3});
    chassis.waitUntil(20);
    tongue.setState(true);
    chassis.waitUntilDone();
    turnToHeading(-90, true);
    leftResetPos();
    moveToPoint(-53, -48, true, false, true);
    turnToHeading(-90, true);
    leftResetPos();
    moveDrive(60, 435);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-28.5, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1100);
    autonWingMacro();
}

inline void leftLGAndC() {
    leftGrabBlocks();
    moveToPoint(-21, 22, false, false, false);
    tongue.setState(false);
    turnToHeading(-45, true);
    moveToPoint(-11, 14, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(900);
    intake.setState(IntakeState::INTAKE);
    chassis.moveToPoint(-43, 48, 1500,  {.minSpeed = 10, .earlyExitRange = 3});
    chassis.waitUntil(20);
    tongue.setState(true);
    chassis.waitUntilDone();
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-53, 48, true, false, true);
    turnToHeading(-90, true);
    rightResetPos();
    moveDrive(60, 300);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-28.5, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(1100);
    autonWingMacro();
}

inline void NISAWP() {
    chassis.setPose(-48, -18, 0);
    moveToPoint(-48, -45, false, false, true, 90);
    tongue.setState(true);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 350);
    waitAtMatchload();
    leftResetPos();
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
    moveToPoint(-17, -17, true, false, true, 50);
    tongue.setState(false);
    moveToPoint(-12, -12, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(1000);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-12);
    intakeLift.setState(false);
    setWings(true);
    moveToPoint(-22, 16, true, true, true, 70);
    tongue.setState(true);
    moveToPoint(-21, 22, true, false, false, 35);
    moveToPoint(-12, 13, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 35);
}

inline void SAWP13() {
    // rightAutoMatchload();
    chassis.setPose(-46.5, 0, 0);
    moveDrive(60, 400);
    intake.setState(IntakeState::INTAKE);
    moveDrive(-10, 150);
    moveToPoint(-47, -45, false, false, true, 90);
    tongue.setState(true);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 440);
    waitAtMatchload();
    leftResetPos();
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
    moveToPoint(-48, 48, true, false, true);
    turnToHeading(-80, true);
    resetPos();
    rightResetPos();
    moveToPoint(-31, 48.5, false, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(950);
    moveToPoint(-50, 48, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(-90, true);
    moveDrive(50, 440);
    waitAtMatchload();
    rightResetPos();
    // moveToPoint(-50, 46, false, false, true, 70);
    moveDrive(-60, 330);
    moveDrive(-10, 75);
    moveToPoint(-17, 14, false, true, true, 85);
    pros::delay(200);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 30);
}

inline void SAWP4Goal() {
    chassis.setPose(-48, -18, 0);
    moveToPoint(-48, -46, false, false, true, 90);
    tongue.setState(true);
    turnToHeading(-90, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 435);
    waitAtMatchload();
    leftResetPos();
    moveToPoint(-28.5, -48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
    turnToHeading(0, true);
    leftResetPos();
    intake.setState(IntakeState::LOWINTAKE);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y + 6, currentPose.theta);
    moveToPoint(-24, -26, true, false, true, 90);
    // tongue.setState(true);
    moveToPoint(-17, -17, true, false, true, 50);
    intake.setState(IntakeState::UPPEROUTTAKE);
    // tongue.setState(false);
    moveToPoint(-11, -11, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(800);
    intake.setState(IntakeState::INTAKE);
    driveDistance(-11);
    intakeLift.setState(false);
    moveToPoint(-20, 17, true, true, true, 90);
    tongue.setState(true);
    moveToPoint(-21, 22, true, false, false, 55);
    moveToPoint(-11, 14, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 35);
    pros::delay(500);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-40, 43.5, true, false, true, 100);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-52, 47, true, false, true);
    turnToHeading(-90, true);
    rightResetPos();
    moveDrive(60, 400);
    waitAtMatchload();
    rightResetPos();
    moveToPoint(-28.5, 48, false, false, true);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    driveToGoal(false);
    goalShake(700);
}

inline void upperMidRush() {
    uint32_t autonStartTime = pros::millis();
    chassis.setPose(-48, 16, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-30, 20, true, false, true);
    tongue.setState(true);
    moveToPoint(-27, 22, true, false, true, 100);
    moveToPoint(-16, 13, false, true, true);
    intake.setState(IntakeState::SCOREMID);
    moveDrive(-20, 300);
    pros::delay(1000);
    middleDescore.setState(true);
    driveDistance(8);
    moveDrive(-45, 300);
    // pros::delay(300);
    leftMotors.move(5);
    rightMotors.move(-25);
    leftDoinker.setState(true);
    intake.setState(IntakeState::IDLE);
    pros::delay(300);
    leftMotors.move(-55);
    rightMotors.move(20);
    while(pros::millis() < autonStartTime + 14000) {
        pros::delay(20);
    }
    moveDrive(25, 250);
    moveDrive(-50, 250);
    leftMotors.move(20);
    rightMotors.move(-20);
    pros::delay(350);
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void midRush() {
    chassis.setPose(-48, -16, 90);
    intake.setState(IntakeState::LOWINTAKE);
    moveToPoint(-30, -20, true, false, true);
    tongue.setState(true);
    moveToPoint(-27, -24, true, false, true, 100);
    turnToHeading(45, true);
    tongue.setState(false);
    intake.setState(IntakeState::UPPEROUTTAKE);
    moveToPoint(-12, -9, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(850);
    intakeLift.setState(false);
    moveToPoint(-24, -15, false, false, true, 80);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 17, true, true, true, 90);
    tongue.setState(true);
    moveToPoint(-21, 22, true, false, true, 55);
    turnToHeading(-45, true);
    moveToPoint(-12, 12, false, false, true);
    intake.setState(IntakeState::SCOREMID);
    leftDoinker.setState(true);
    driveToGoal(false, 35);
    pros::delay(800);
    middleDescore.setState(true);
    driveDistance(6);
    driveToGoal(false);
    leftMotors.move(-40);
    rightMotors.move(-5);
}

inline void midRushExtraBlocks() {
    rightGrabBlocks();
    intake.setState(IntakeState::LOWINTAKEUPOUTTAKE);
    moveToPoint(-24, -26, false, false, true);
    tongue.setState(false);
    turnToHeading(45, true);
    intake.setState(IntakeState::UPPEROUTTAKE);
    moveToPoint(-13, -13, true, false, true, 70);
    intake.setState(IntakeState::SCORELOW);
    intakeLift.setState(true);
    pros::delay(850);
    intakeLift.setState(false);
    moveToPoint(-21, -15, false, false, true, 80);
    turnToHeading(0, true);
    leftResetPos();
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-21, 17, true, false, true, 90);
    tongue.setState(true);
    moveToPoint(-21, 22, true, false, true, 55);
    turnToHeading(-45, true);
    moveToPoint(-14, 11, false, false, true);
    intake.setState(IntakeState::SCOREMID);
    leftDoinker.setState(true);
    driveToGoal(false, 35);
    pros::delay(800);
    middleDescore.setState(true);
    driveDistance(6);
    driveToGoal(false);
    leftMotors.move(-40);
    rightMotors.move(-5);
}

inline void sugarSkillsAuton() {
    chassis.setPose(-46.5, 8, 90);
    intake.setState(IntakeState::INTAKE);
    moveToPoint(-24, 24, true, false, false, 70);
    tongue.setState(true);
    pros::delay(300);
    moveToPoint(-12, 12, false, true, true, 70);
    intake.setState(IntakeState::SCOREMID);
    driveToGoal(false, 40);
    pros::delay(600);
    intake.setState(IntakeState::IDLE);
    moveToPoint(-34, 48, true, false);
    middleGoal.setState(true);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-50, 48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(65, 500);
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
    moveToPoint(49, 48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
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
    moveDrive(60, 300);
    moveToPoint(48, -24, false, false);
    turnToHeading(90, true);
    resetPos();
    rightResetPos();
    moveToPoint(19, -17, false, true);
    turnToHeading(135);
    driveToGoal(false, 40);
    tongue.setState(true);
    intake.setState(IntakeState::SCOREMID);
    pros::delay(4100);
    intake.setState(IntakeState::IDLE);
    middleGoal.setState(true); ////////////////////// SCORED 7
    moveToPoint(38, -48, true, false);
    turnToHeading(90, true);
    rightResetPos();
    moveToPoint(50, -48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(65, 500);
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
    moveToPoint(-49,-48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
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
    moveDrive(60, 700);
    moveDrive(-2, 200);
    driveInParkZone();
}


inline void skillsAuton(){
    //-----------------------Park zone + Middle Goal 7-----------------------
    chassis.setPose(-55, 0, -90);
    odomLift.setState(true);
    // tongue.setState(true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(15, 300);
    // tongue.setState(false);
    pros::delay(600);
    moveDrive(50, 400);
    moveDrive(0, 1100);
    // intakeShake(1100);
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
    pros::delay(2700);
    // slowMidGoal = true;
    // pros::delay(500);
    //-----------------------1st Matchload -----------------------
    chassis.moveToPoint(-42, 45, 1200, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntil(8);
    intake.setState(IntakeState::INTAKE);
    tongue.setState(true);
    turnToHeading(-90, true);
    rightResetPos();
    moveToPoint(-52, 47, true, false, true);
    turnToHeading(-90, true);
    rightResetPos();
    moveDrive(65, 500);
    skillsWaitAtMatchload(); /////////// FIRST MATCHLOAD
    rightResetPos();
	leftMotors.move(-80);
	rightMotors.move(-20);
	pros::delay(200);
    turnToHeading(40, true);
    leftMotors.move(80);
	rightMotors.move(40);
	pros::delay(300);
    moveDrive(105, 800);
    // rightResetPos();
    resetPos();
    leftResetPos();
    moveToPoint(31, 66, true, false, true, 90);
    turnToHeading(35, true);
    turnWithSpeeds(-30, -90, 75);
    //-----------------------1st Long Goal -----------------------
    moveDrive(-50, 300);
    intake.setState(IntakeState::SCOREHIGH);
    goalShake(2000);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(28, currentPose.y, currentPose.theta);
    turnToHeading(90, true);
    leftResetPos();
    moveToPoint(49, 48, true, false, true);
    turnToHeading(90, true);
    intake.setState(IntakeState::INTAKE);
    moveDrive(60, 500);
    //-----------------------2nd Matchload -----------------------
    skillsWaitAtMatchload();
    leftResetPos();
    moveToPoint(40, 48, false, false, true);
    moveToPoint(30, 48, false, false, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    pros::delay(500);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(1200);
    currentPose = chassis.getPose();
    chassis.setPose(28, currentPose.y, currentPose.theta);
    // --------------- GOING TO CLEAR PARK ZONE
    moveToPoint(62, 36, true, false, true);
    intake.setState(IntakeState::INTAKE);
    turnToHeading(168);
    moveDrive(60, 550);
    moveDrive(-2, 200);
    skillsClearParkZone();
	// tongue.setState(true);
    leftResetPos();
    leftMotors.move(40);
    rightMotors.move(55);
    pros::delay(380);
    leftMotors.move(0);
    rightMotors.move(0);
    moveDrive(-70, 150);
	chassis.turnToHeading(145, 500, {.minSpeed = 80, .earlyExitRange = 20});
    turnToHeading(-165, true);
	leftResetPos();
    resetPos();
	moveToPoint(27, -19.5, true, true, false, 80);
	moveToPoint(17, 18, true, true, false, 80);
	moveToPoint(12, 12, true, true, true, 80);
     //-----------------------Score 7 Low -----------------------
    intakeLift.setState(true);
    intake.setState(IntakeState::SCORELOW);
	driveToGoal(true);
    pros::delay(3200);
	moveDrive(-60, 250);
	intakeLift.setState(false);
	intake.setState(IntakeState::INTAKE);
    turnToPoint(38, -42, true);
    chassis.moveToPoint(38, -42, 1500, {.minSpeed = 5, .earlyExitRange = 2});
    chassis.waitUntil(16);
    tongue.setState(true);
	turnToHeading(90, false);
	rightResetPos();
	moveToPoint(52, -47, true, true);
	turnToHeading(90, true);
	rightResetPos();
    moveDrive(65, 500);
	//-----------------------THIRD MATCHLOAD -----------------------
    skillsWaitAtMatchload(); /////////// THIRD MATCHLOAD
    rightResetPos();
	leftMotors.move(-80);
	rightMotors.move(-20);
	pros::delay(200);
    turnToHeading(-150, true);
    leftMotors.move(80);
	rightMotors.move(40);
	pros::delay(300);
    moveDrive(105, 800);
    resetPos();
    leftResetPos();
    moveToPoint(-30,-68, true, false, true, 90);
    turnToHeading(-145, true);
    turnWithSpeeds(-30, -90, -105);
    //-----------------------4th long goal -----------------------
    moveDrive(-50, 300);
    intake.setState(IntakeState::SCOREHIGH);
	goalShake(2000);
    turnToHeading(-90, true);
    // leftResetPos();
    currentPose = chassis.getPose();
    chassis.setPose(-28, -48, currentPose.theta);
    moveToPoint(-49, -48, true, false, true);
    turnToHeading(-90, true);
    intake.setState(IntakeState::INTAKE);
	// leftResetPos();
    // turnToHeading(-90, true);
    // moveToPoint(-55,-52, true, false, true);
    // turnToHeading(-90, true);
    // intake.setState(IntakeState::INTAKE);
    moveDrive(60, 350);
    skillsWaitAtMatchload(); /////// FOURTH MATCHLOAD
    leftResetPos();
    // moveToPoint(-40, -48, false, false, true);
    moveToPoint(-35, -48, false, false, true);
    turnToHeading(-90, true);
    driveToGoal(false);
    tongue.setState(false);
    intake.setState(IntakeState::SCOREHIGH);
    goalShake(800);
    intake.setState(IntakeState::SCOREHIGHSLOW);
    pros::delay(700);
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
        // sugarSkillsAuton();
        allianceColour = AllianceColour::RED;
        enableColourSort = false;
    }
    if(startingSide == Selection::LEFT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            // left7Block(); // not long control
            left7BlockWing();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            // leftLGAndC_CenterLast();
            leftLGAndCRush();
        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::LONG_CONTROL) {
            // right7Block(); // not long control
            // right7BlockWing();
            right7Block();
        }
        if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
            // rightLGAndC_CenterLast();
            rightLGAndCRush();
        }
        if(autonSelectionEnum == Selection::SOLO_AWP) {
            SAWP13();
        }
        if(autonSelectionEnum == Selection::NI_SAWP) {
            SAWP4Goal();
        }
    }
}