#include "autons.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"

ASSET(skillsPath1_txt);
ASSET(skillsPath2_txt);
ASSET(skillsPath3_txt);
ASSET(skillsPath4_txt);
ASSET(skillsPath5_txt);
ASSET(skillsPath6_txt);
ASSET(skillsPath7_txt);

ASSET(mogoRushBlue1_txt); 

ASSET(nonMogoRushRed1_txt);

void lineUpWallStake(bool turnDirection) {
    leftMotors.move(20);
    rightMotors.move(20);
    while(lineTracker.get_value() > 2000) {
        pros::delay(20);
    }
	leftMotors.move(0);
	rightMotors.move(0);
    if(turnDirection) {
        currentPose = chassis.getPose();
        chassis.setPose(1, currentPose.y, currentPose.theta);
        chassis.turnToHeading(180, 3000);
    } else {
        currentPose = chassis.getPose();
        chassis.setPose(1, currentPose.y, currentPose.theta);
        chassis.turnToHeading(0, 3000);
    }
    chassis.waitUntilDone();
}

void skillsAuton() {
    // set position to x:0, y:0, heading:0
    chassis.setPose(-61, 0, 90); 
    intake.move(127);
	pros::delay(300);   
	chassis.turnToHeading(-30, 1200, {.minSpeed = 80, .earlyExitRange = 10});
	chassis.follow(skillsPath1_txt, 12, 3000, false);
	chassis.waitUntil(30);
	mogoMech.set_value(true);
    pros::delay(300);
    chassis.follow(skillsPath2_txt, 15, 3500);
    chassis.waitUntilDone();
    chassis.moveToPoint(10, -57, 2000, {.minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 60, .earlyExitRange = 10});
    while(wallStake.get_position() < 240) {
        wallStake.move(35);
        pros::delay(20);
    } 
    wallStake.move(2);
    lineUpWallStake(true);
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, -61.5, 2500);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallStake.move(2);
    while(intake.get_actual_velocity() > 15) {
        pros::delay(20);
    }
    intake.move(-10);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(180, 2000);
    exitLoop = false;
    while(wallStake.get_position() < 1400 && !exitLoop) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                exitLoop = true;
                intake.move(-127);
            }
        }
    }

    //SCORED THE FIRST WALLSTAKE!! and scored the first two rings on mogooo

    currentPose = chassis.getPose();
    chassis.setPose(0, -61.5, currentPose.theta);
    wallStake.move(-127);
    chassis.moveToPoint(0, -53, 2000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(127);
    wallStake.move(0);
    chassis.turnToHeading(-90, 1500, {.minSpeed = 80, .earlyExitRange = 15});
    intake.move(127);
    chassis.follow(skillsPath3_txt, 15, 6000);
    chassis.moveToPoint(-48, -60, 3000, {.earlyExitRange=10});
    chassis.moveToPoint(-55, -65, 3000, {.forwards=false, .minSpeed=30});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    pros::delay(300);
    intake.move(-127);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x -4, currentPose.y, currentPose.theta);
    chassis.follow(skillsPath4_txt, 18, 2000);
    chassis.turnToPoint(-48, 24, 2000, {.minSpeed = 60, .earlyExitRange = 30});
    chassis.moveToPoint(-48, 24, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(127);
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(90, 2000, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.moveToPoint(-30, 24, 2000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(0, 40, 2000, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.moveToPoint(24, 48, 2000, {.earlyExitRange = 5});
    //chassis.follow(skillsPath5_txt, 16, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(10, 55, 2000, {.minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 60, .earlyExitRange = 10});
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(wallStake.get_position() < 240) {
        wallStake.move(35);
        pros::delay(20);
        std::cout <<wallStake.get_position();
    }
    wallStake.move(2);
    lineUpWallStake(false);

    //lining up wall stake!

    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, 61.5, 2500);
    while(intake.get_actual_velocity() > 5) {
        pros::delay(20);
    }
    intake.move(-10);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(0, 2000);
    while(wallStake.get_position() < 1400) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                intake.move(-127);
                break;
            }
        }
    }

    //SCORED THE SECOND WALLSTAKE!!    
    
    currentPose = chassis.getPose();
    chassis.setPose(0, 61.5, currentPose.theta);
    wallStake.move(-127);
    chassis.moveToPoint(0, 53, 2000, {.forwards = false});
    chassis.follow(skillsPath6_txt, 15, 3500);
    chassis.moveToPoint(-48, 60, 3000, {.earlyExitRange=10});
    chassis.moveToPoint(-55, 63, 3000, {.forwards=false, .minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(600);
    mogoMech.set_value(false);
    pros::delay(200);
    //chassis.follow(skillsPath7_txt, 15, 5000);
}


void mogoRushBlue() {
    chassis.setPose(52, -62.5, 90);
    chassis.follow(mogoRushBlue1_txt, 15, 3000, false);
    chassis.turnToHeading(90, 300, {.earlyExitRange = 10});
    chassis.moveToPoint(10.5, -46, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    intake.move(127);
    doinker.set_value(LOW);
    chassis.moveToPoint(55.5, -63, 2500, {.earlyExitRange = 5});
    chassis.turnToHeading(0, 1500, {.minSpeed = 60, .earlyExitRange = 30});
    chassis.waitUntilDone();
    doinker.set_value(HIGH);
    chassis.turnToHeading(135, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(63, -63, 1500, {.minSpeed = 60});
    chassis.moveToPoint(52, -52, 1500, {.minSpeed = 60, .earlyExitRange = 8});
    chassis.turnToHeading(-90, 1500, {.earlyExitRange = 15});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    chassis.moveToPoint(40, -53, 1500, {.earlyExitRange = 10});
    chassis.moveToPoint(17, -39, 1500);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.turnToPoint(24, -24, 500, {.forwards = false, .earlyExitRange = 15});
    chassis.moveToPoint(23, -25, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(350);
    intake.move(127);
    pros::delay(500);
    chassis.moveToPoint(17, -17, 1500, {.forwards = false});
    while(intake.get_actual_velocity() > 20) {
        pros::delay(20);
    }
    intake.move(10);
}

void nonMogoRushRed() {
    chassis.setPose(-52.25, 24, -90);
    chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(20, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPose(-11, 42, 60, 1500);
    chassis.turnToHeading(0, 1500);
    chassis.moveToPoint(-11, 47, 1500);
    chassis.moveToPoint(-12, 36, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 8});
    chassis.moveToPoint(-24, 26, 2000, {.forwards = false});
    chassis.moveToPoint(-26, 50, 1500);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    chassis.turnToHeading(80, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-56, 24, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 20});
    chassis.moveToPoint(-60, -3, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-60, 0, 2000, {.earlyExitRange = 3});
    chassis.turnToPoint(-70, 0, 2000, {.forwards = false});
    chassis.moveToPoint(-65.5, 0, 2000, {.forwards = false, .minSpeed = 30});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(700);
    intake.move(0);
    chassis.swingToHeading(90,lemlib::DriveSide::RIGHT, 1500, {.earlyExitRange = 15});
    chassis.moveToPoint(-48, 24, 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    while(wallStake.get_position() < 700) {
        wallStake.move(60);
        pros::delay(20);
    }
    wallStake.move(0);
    chassis.moveToPoint(-12,15, 1500, {.forwards = false});
    chassis.waitUntilDone();
    wallStake.move(-127);
}

void nonMogoRushBlue() {
    chassis.setPose(52.25, 24, 90);
    chassis.moveToPoint(24, 24, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-20, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPose(12, 42, 60, 1500);
    chassis.turnToHeading(0, 1500);
    chassis.moveToPoint(12, 47, 1500);
    chassis.moveToPoint(13, 36, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 8});
    chassis.moveToPoint(24, 26, 2000, {.forwards = false});
    chassis.moveToPoint(26, 48, 1500);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    chassis.turnToHeading(-80, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(56, 24, 2000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 20});
    chassis.moveToPoint(60, -3, 2000, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(60, 0, 2000, {.earlyExitRange = 3});
    chassis.turnToPoint(70, 0, 2000, {.forwards = false});
    chassis.moveToPoint(65, 0, 1000, {.forwards = false, .minSpeed = 30});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(700);
    intake.move(0);
    chassis.swingToHeading(-90,lemlib::DriveSide::LEFT, 1500, {.earlyExitRange = 15});
    chassis.moveToPoint(48, 24, 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    while(wallStake.get_position() < 700) {
        wallStake.move(60);
        pros::delay(20);
    }
    wallStake.move(0);
    chassis.waitUntilDone();
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.moveToPoint(12,15, 1500, {.forwards = false});
    chassis.waitUntilDone();
    wallStake.move(-127);
}

void mogoRushRed() {
    
}