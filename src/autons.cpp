#include "autons.hpp"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

void autonCheckColour() {
	if (colourSortingSensor.get_hue() > 80 && colourSortingSensor.get_hue() < 300 && colourSortingSensor.get_proximity() > 200) {
		ringColour = true; //blue
	}
	else if (colourSortingSensor.get_hue() < 20 || colourSortingSensor.get_hue() > 320 && colourSortingSensor.get_proximity() > 200){
		ringColour = false; //red
	}
}

void multitaskIntakeControl_f(void* param) {
    bool autonColourSortingEjectionState = false;
    int ejectCounter = 0;
    while(true) {
        if(multitaskIntakeControls) {
            autonCheckColour();
            intake.move(127);
            intakeThreadCounter++;
            if(intakeThreadCounter > 50) {
                if(intake.get_actual_velocity() < 3) {
                    intake.move(-127);
                    intakeThreadCounter = 0;
                }
            }
        } else {
            intakeThreadCounter = 0;
        }
        if(colourSortingSelector == 0) {

		} else if (colourSortingSelector == 1) { //red
			if(ringColour == false && distanceSort.get() < 200) {
				autonColourSortingEjectionState = true;
			}
		} else if (colourSortingSelector == 2) { //blue
			if(ringColour == true && distanceSort.get() < 200) {
				autonColourSortingEjectionState = true;
			}
        }

        if(autonColourSortingEjectionState) {
            intake.move(-10);
			ejectCounter++;
			if (ejectCounter > 8) {
				autonColourSortingEjectionState = false;
				ejectCounter = 0;
			}
		}
        pros::delay(20);
    }
}

inline pros::Task multitaskIntakeControl(multitaskIntakeControl_f, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "multitaskIntakeControl");

ASSET(skillsPath1_txt);
ASSET(skillsPath2_txt);
ASSET(skillsPath3_txt);
ASSET(skillsPath4_txt);
ASSET(skillsPath5_txt);
ASSET(skillsPath6_txt);
ASSET(skillsPath7_txt);
ASSET(skillsPath8_txt);

ASSET(mogoRushBlue1_txt); 

ASSET(nonMogoRushRed1_txt);

ASSET(mogoRushRed1_txt); 

void testingAuton() {
    multitaskIntakeControls = false;
    while(true) {
        pros::delay(20);
    }
}

void lineUpWallStake(bool turnDirection) {
    leftMotors.move(23);
    rightMotors.move(23);
    leftMotors.move(23);
    rightMotors.move(23);
    leftMotors.move(23);
    rightMotors.move(23);
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
    pros::delay(200);
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
    chassis.moveToPoint(15, -57, 2000, {.minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 60, .earlyExitRange = 10});
    while(wallStake.get_position() < 215) {
        wallStake.move(35);
        pros::delay(20);
    } 
    wallStake.move(2);
    lineUpWallStake(true);
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, -61.5, 2500);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallStake.move(2);
    int counter = 0;
    while(intake.get_actual_velocity() > 10 && counter < 250) {
        pros::delay(20);
        counter++;
    }
    intake.move(-10);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(180, 2000);
    exitLoop = false;
    while(wallStake.get_position() < 1450 && !exitLoop) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                exitLoop = true;
            }
        }
    }
    intake.move(-127);

    //SCORED THE FIRST WALLSTAKE!! and scored the first two rings on mogooo

    currentPose = chassis.getPose();
    wallStake.move(-127);
    chassis.moveToPoint(0, -53, 2000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(127);
    wallStake.move(0);
    chassis.turnToHeading(-90, 1500, {.minSpeed = 80, .earlyExitRange = 15});
    intake.move(127);
    chassis.follow(skillsPath3_txt, 15, 6000);
    chassis.moveToPoint(-48, -60, 3000, {.earlyExitRange=10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-55, -65, 3000, {.forwards=false, .minSpeed=30});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    pros::delay(300);
    intake.move(-127);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
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
    chassis.moveToPoint(15, 56, 2000, {.minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 60, .earlyExitRange = 10});
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(wallStake.get_position() < 215) {
        wallStake.move(35);
        pros::delay(20);
    }
    wallStake.move(2);
    controller.rumble("-");
    pros::screen::print(TEXT_LARGE_CENTER, 1, "I HAVE BEEN AWAKENED");
    lineUpWallStake(false);

    //lining up wall stake!

    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, 61.5, 1000);
    counter = 0;
    while(intake.get_actual_velocity() > 10 && counter<250) {
        pros::delay(20);
        counter++;
    }
    intake.move(-10);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(0, 2000);
    while(wallStake.get_position() < 1450) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                break;
            }
        }
    }
    intake.move(-127);

    //SCORED THE SECOND WALLSTAKE!!    
    
    currentPose = chassis.getPose();
    wallStake.move(-127);
    chassis.moveToPoint(0, 53, 2000, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPoint(-26, 47, 2000, {.minSpeed = 50});
    chassis.moveToPoint(-53, 48, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(-48, 61, 3000, {.earlyExitRange=10});
    chassis.moveToPoint(-56, 63, 3000, {.forwards=false, .minSpeed=30});
    chassis.moveToPoint(-56, 63, 3000, {.forwards=false, .minSpeed=30});
    chassis.waitUntilDone();
    pros::delay(600);
    mogoMech.set_value(false);
    intake.move(0);
    pros::delay(200);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x+3, currentPose.y+1, currentPose.theta);
    skillsAutonSecondHalf();
}

void skillsAutonSecondHalf() {
    //chassis.setPose(-55, 63, 90);
    chassis.follow(skillsPath7_txt, 15, 6000);
    chassis.waitUntilDone();
    intake.move(70);
    chassis.moveToPoint(58, 24, 2500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(63, 57, 2500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(55, 25, 2000, {.forwards = false, .minSpeed=50});
    chassis.moveToPoint(48, 3, 2000, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y -3, currentPose.theta);
    chassis.moveToPoint(24, 42, 2000, {.earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnToPoint(44, 44, 1500, {.minSpeed = 25, .earlyExitRange = 5});
    chassis.moveToPoint(44, 44, 1500, {.minSpeed = 30});
    chassis.moveToPoint(38, 48, 1500, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(45, 54, 1500, {.minSpeed = 30});
    chassis.follow(skillsPath8_txt, 16, 3000, false);
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x-4, currentPose.y -4, currentPose.theta);
    chassis.turnToPoint(24, -24, 1500, {.minSpeed = 30});
    chassis.moveToPoint(26, -24, 2000);
    chassis.moveToPoint(24, -52, 2000, {.minSpeed = 30, .earlyExitRange = 10});
    chassis.moveToPoint(41, -65, 2000);
    chassis.turnToPoint(43, -52, 1500);
    chassis.moveToPoint(43, -52, 1500);
    chassis.turnToHeading(-45, 1500);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    pros::delay(300);
    intake.move(-90);
    chassis.moveToPoint(58, -60, 1500, {.forwards = false});
    chassis.moveToPoint(36, -36, 2000, {.minSpeed = 30, .earlyExitRange = 10});
    chassis.moveToPoint(0, 0, 3000, {.forwards = false,.maxSpeed = 70, .minSpeed = 30});
    while(wallStake.get_position() < 1500) {
        wallStake.move(80);
    }
    wallStake.move(0);
}


void mogoRushBlue() {
    chassis.setPose(52, -61, -90);
    colourSortingSelector = 0;
    multitaskIntakeControls = false;
    chassis.follow(mogoRushBlue1_txt, 15, 3000);
    chassis.waitUntil(18);
    intake.move(70);
    chassis.turnToPoint(10,-49, 300, {.forwards=false,.earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(9, -49, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    colourSortingSelector=0;
    multitaskIntakeControls = true;
    chassis.moveToPoint(34, -49, 1500);
    chassis.waitUntilDone();
    pros::delay(500);
    mogoMech.set_value(false);
    multitaskIntakeControls = false;
    intake.move(0);
    chassis.turnToPoint(28, -24, 500, {.forwards = false});
    chassis.moveToPoint(24, -24, 1500, {.forwards = false, .maxSpeed=80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(57, -39, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(0, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    multitaskIntakeControls=true;
    chassis.moveToPoint(52,-28, 800, {.minSpeed=127});
    chassis.moveToPoint(57, -34, 1500, {.forwards=false});
    chassis.turnToPoint(48, -48, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    multitaskIntakeControls=false;
    intake.move(-127);
    chassis.moveToPoint(46, -56, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.swingToPoint(62,-62, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    wallStake.move(0);
    chassis.waitUntilDone();
    colourSortingSelector = 1;
    multitaskIntakeControls=true;
    chassis.moveToPoint(64,-64, 1500, {.minSpeed =70});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(15, -15, 1500, {.forwards = false});
    pros::delay(700);
    multitaskIntakeControls = false;
    intake.move(-127);
    while(wallStake.get_position() < 320) {
        wallStake.move(127);
    }
    wallStake.move(3);
    mogoState = true;
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
    chassis.moveToPoint(-67, 3, 500, {.forwards = false, .minSpeed = 30});
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
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
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
    chassis.moveToPose(14, 42, 50, 1500);
    chassis.turnToHeading(0, 1500);
    chassis.moveToPoint(14, 47, 1500);
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
    chassis.moveToPoint(60, 3, 2000, {.earlyExitRange = 3});
    chassis.turnToPoint(70, 3, 2000, {.forwards = false});
    chassis.moveToPoint(65, 3, 1000, {.forwards = false, .minSpeed = 30});
    chassis.waitUntilDone();
    intake.move(127);
    pros::delay(700);
    intake.move(0);
    chassis.swingToHeading(-90,lemlib::DriveSide::LEFT, 1500, {.earlyExitRange = 15});
    chassis.moveToPoint(48, 24, 2500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(wallStake.get_position() < 700) {
        wallStake.move(60);
        pros::delay(20); 
    }
    wallStake.move(0);
    chassis.moveToPoint(12,12, 1500, {.forwards = false});
    chassis.waitUntilDone();
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.waitUntilDone();
    wallStake.move(-127);
}

void mogoRushRed() {
    chassis.setPose(-52, -61, 90);
    colourSortingSelector = 0;
    multitaskIntakeControls = false;
    chassis.follow(mogoRushRed1_txt, 15, 3000);
    chassis.waitUntil(18);
    intake.move(70);
    chassis.turnToPoint(-10,-49, 300, {.forwards=false,.earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(-9, -49, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    colourSortingSelector=0;
    multitaskIntakeControls = true;
    chassis.moveToPoint(-34, -49, 1500);
    chassis.waitUntilDone();
    pros::delay(600);
    mogoMech.set_value(false);
    multitaskIntakeControls = false;
    intake.move(0);
    chassis.turnToPoint(-24, -24, 500, {.forwards = false});
    chassis.moveToPoint(-24, -24, 1500, {.forwards = false, .maxSpeed=80});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-57, -39, 1500);
    chassis.turnToHeading(0, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls=true;
    chassis.moveToPoint(-52,-28, 800, {.minSpeed=127});
    chassis.moveToPoint(-57, -34, 1500, {.forwards=false});
    chassis.turnToPoint(-48, -48, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls=false;
    intake.move(-127);
    chassis.moveToPoint(-46, -56, 1500);
    chassis.swingToPoint(-62,-62, lemlib::DriveSide::LEFT, 1500);
    wallStake.move(0);
    chassis.waitUntilDone();
    colourSortingSelector = 1;
    multitaskIntakeControls=true;
    chassis.moveToPoint(-64,-64, 1500, {.minSpeed =70});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.moveToPoint(-18, -18, 1500, {.forwards = false});
    pros::delay(700);
    multitaskIntakeControls = false;
    intake.move(-127);
    while(wallStake.get_position() < 320) {
        wallStake.move(127);
    }
    wallStake.move(3);
    mogoState = true;
}