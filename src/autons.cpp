#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
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

// void multitaskIntakeControl_f(void* param) {
//     bool autonColourSortingEjectionState = false;
//     int ejectCounter = 0;
//     while(true) {
//         if(multitaskIntakeControls) {
//             autonCheckColour();
//             intake.move(127);
//             intakeThreadCounter++;
//             if(intakeThreadCounter > 50) {
//                 if(intake.get_actual_velocity() < 3) {
//                     intake.move(-127);
//                     intakeThreadCounter = 0;
//                 }
//             }
//         } else {
//             intakeThreadCounter = 0;
//         }
//         if(colourSortingSelector == 0) {

// 		} else if (colourSortingSelector == 1) { //red
// 			if(ringColour == false && distanceSort.get() < 200) {
// 				autonColourSortingEjectionState = true;
// 			}
// 		} else if (colourSortingSelector == 2) { //blue
// 			if(ringColour == true && distanceSort.get() < 200) {
// 				autonColourSortingEjectionState = true;
// 			}
//         }

//         if(autonColourSortingEjectionState) {
//             intake.move(-10);
// 			ejectCounter++;
// 			if (ejectCounter > 8) {
// 				autonColourSortingEjectionState = false;
// 				ejectCounter = 0;
// 			}
// 		}
//         pros::delay(20);
//     }
// }

//inline pros::Task multitaskIntakeControl(multitaskIntakeControl_f, nullptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "multitaskIntakeControl");

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

void driveToDistance(double distance, int maxSpeed, bool resetPos) {
    double kP = 3;
    double error;
    leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    double disToWall = distanceToWall.get_distance() / 25.4; // convert mm to inches
    while(disToWall > distance) {
        disToWall = distanceToWall.get_distance() / 25.4;
        error = disToWall - distance;
        if(error*kP > maxSpeed) {
            leftMotors.move(maxSpeed);
            rightMotors.move(maxSpeed);
        } else {
            leftMotors.move(error * kP + 15);
            rightMotors.move(error * kP + 15);
        }
        pros::screen::print(TEXT_LARGE_CENTER, 7, "distance: %i\n", distanceToWall.get());
        pros::delay(20);
        pros::screen::erase();
    }
    leftMotors.move(0);
    rightMotors.move(0);
    if(resetPos) {
        lemlib::Pose currentPose = chassis.getPose();
        double absoluteTheta = (int)currentPose.theta % 360;
        if(fabs(absoluteTheta - 0) < 8) {
            chassis.setPose(currentPose.x, 71 - disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta - 90) < 8) {
            chassis.setPose(71 - disToWall, currentPose.y, currentPose.theta);
        } else if(fabs(absoluteTheta - 180) < 8 || fabs(currentPose.theta + 180) < 8) {
            chassis.setPose(currentPose.x, -71 + disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta + 90) < 8) {
            chassis.setPose(-71 + disToWall, currentPose.y, currentPose.theta);
        }
    }
}

void driveDistance(double distance, double maxSpeed) {
    double kP = 4.5;
    double error;
    double rotationsToInches = 8.164;
    leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    double startingPos = rightMotors.get_position();
    double destinationPos = startingPos * rotationsToInches + distance;
    double motorSpeed;
    if(distance > 0) {
        while(rightMotors.get_position() * rotationsToInches < destinationPos) {
            error = destinationPos - rightMotors.get_position() * rotationsToInches;
            if(error * kP + 30 > maxSpeed) {
                motorSpeed = maxSpeed; 
            } else {
                motorSpeed = error * kP + 30;
            }
            if(motorSpeed > (rightMotors.get_position() - startingPos) * 360) {
                motorSpeed = (rightMotors.get_position() - startingPos) * 360 + 50;
            }
            leftMotors.move(motorSpeed);
            rightMotors.move(motorSpeed);
            pros::screen::print(TEXT_LARGE_CENTER, 1, "currentPos: %f\n", rightMotors.get_position()* rotationsToInches);
            //pros::screen::print(TEXT_LARGE_CENTER, 10, "destinationPos: %d", destinationPos);
        }
        leftMotors.move(0);
        rightMotors.move(0);
    } if(distance < 0) {
        while(leftMotors.get_position() / 10.205 / 360 > destinationPos) {
            error = destinationPos - leftMotors.get_position() / 10.205 / 360;
            if(error * kP - 0 < maxSpeed) {
                leftMotors.move(maxSpeed);
                rightMotors.move(maxSpeed);
            } else {
                leftMotors.move(error * kP + 0);
                rightMotors.move(error * kP + 0);
            }
        }
        leftMotors.move(0);
        rightMotors.move(0);
    }
}

void testingAuton() {
    multitaskIntakeControls = false;
    while(true) {
        pros::delay(20);
    }
}

void lineUpWallStake(bool turnDirection) {
    mogoMech.set_value(true);
    pros::delay(200);
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
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(500);
}

void skillsAuton() {
    colourSortingSelector = 2;
    ringColour = false;
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
    chassis.moveToPoint(18, -56.5, 2000, {.minSpeed = 20, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 20});
    while(wallStake.get_position() < 240) {
        wallStake.move(35);
        pros::delay(20);
    } 
    wallStake.move(2);
    chassis.waitUntilDone();
    lineUpWallStake(true);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wallStake.move(2);
    int counter = 0;
    while(intake.get_actual_velocity() > 10 && counter < 150) {
        pros::delay(20);
        counter++;
    }
    intake.move(-20);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    exitLoop = false;
    while(wallStake.get_position() < 1500 && !exitLoop) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                exitLoop = true;
            }
        }
        if(wallStake.get_position() > 1300) {
            leftMotors.move(-7);
            rightMotors.move(-7);
        }
    }
    intake.move(-127);
    //SCORED THE FIRST WALLSTAKE!! and scored the first two rings on mogooo

    currentPose = chassis.getPose();
    wallStake.move(-127);
    chassis.moveToPoint(0, -53, 2000, {.forwards = false});
    chassis.waitUntilDone();
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
    chassis.moveToPoint(18, 51.5, 2000, {.minSpeed = 60, .earlyExitRange = 3});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 20, .earlyExitRange = 2});
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(wallStake.get_position() < 235) {
        wallStake.move(35);
        pros::delay(20);
    }
    wallStake.move(2);
    controller.rumble("-");
    pros::screen::print(TEXT_LARGE_CENTER, 1, "I HAVE BEEN AWAKENED");
    chassis.waitUntilDone();
    lineUpWallStake(false);

    //lining up wall stake!
    counter = 0;
    while(intake.get_actual_velocity() > 10 && counter<150) {
        pros::delay(20);
        counter++;
    }
    intake.move(-10);
    wallStake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.turnToHeading(0, 2000);
    while(wallStake.get_position() < 1500) {
        wallStake.move(127);
        pros::delay(20);
        if(wallStake.get_position() > 500) {
            if(wallStake.get_actual_velocity() < 10) {
                break;
            }
        }
        if(wallStake.get_position() > 1300) {
            leftMotors.move(-7);
            rightMotors.move(-7);
        }
    }
    intake.move(-127);

    //SCORED THE SECOND WALLSTAKE!!    
    
    currentPose = chassis.getPose();
    wallStake.move(-127);
    chassis.moveToPoint(0, 53, 2000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30, .earlyExitRange = 10});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPoint(-26, 47, 2000, {.minSpeed = 50});
    chassis.moveToPoint(-56, 48, 2000, {.maxSpeed = 80});
    chassis.moveToPoint(-48, 61, 3000, {.minSpeed = 30, .earlyExitRange=10});
    chassis.turnToHeading(95, 1000, {.minSpeed = 30});
    chassis.moveToPoint(-54, 65 , 1000, {.forwards=false, .minSpeed=50});
    chassis.waitUntilDone();
    pros::delay(1200);
    mogoMech.set_value(false);
    intake.move(0);
    pros::delay(200);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x+2, currentPose.y+1, currentPose.theta);
    skillsAutonSecondHalf();
}

void skillsAutonSecondHalf() {
    //chassis.setPose(-55, 63, 90);
    chassis.follow(skillsPath7_txt, 15, 6000);
    chassis.waitUntilDone();
    intake.move(70);
    chassis.moveToPoint(61, 24, 2500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(63, 58.5, 2500, {.minSpeed = 50, .earlyExitRange = 5});
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
    chassis.moveToPoint(39, 54, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(180, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.follow(skillsPath8_txt, 13, 3000);
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x-3, currentPose.y-3, currentPose.theta);
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
    chassis.moveToPoint(0, 0, 3000, {.forwards = false,.maxSpeed = 60, .minSpeed = 30});
    while(wallStake.get_position() < 1500) {
        wallStake.move(80);
    }
    wallStake.move(0);
    pros::delay(5000);
    doinker.set_value(true);
}


void mainPosBlue() {
    chassis.setPose(52, -59.75, -76);
    wallStake.move(30);
    firstStageIntake.move(127);
	driveDistance(31.5, 127);
    int count = 0;
    while(LB_rotation.get_position() / 100 < 205) {
        wallStake.move(127);
        count += 1;
        if (count > 65) {
            break;
        }
        pros::delay(20);
    }
    wallStake.move(-60);
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
    wallStake.move(-30);
    chassis.turnToHeading(-135, 1500);
    chassis.waitUntilDone();
    wallStake.move(-127);
    secondStageIntake.move(-50);
    chassis.moveToPoint(26, -48, 1500, {.forwards = false});
    secondStageIntake.move(0);
    firstStageIntake.move(127);
    wallStake.move(0);
    chassis.moveToPoint(12, -44, 1000);
    chassis.moveToPoint(24, -28, 1000, {.forwards = false});
    chassis.moveToPoint(28, -24, 1000, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(true); 
    chassis.moveToPoint(44, -65, 1500);
    chassis.turnToHeading(110, 1000);
    chassis.waitUntilDone();
    doinker.set_value(true);
    secondStageIntake.move(127);
    chassis.moveToPoint(59, -67, 1500);
    chassis.turnToHeading(0, 1500);
    chassis.waitUntilDone();
    doinker.set_value(false);
}

void nonMogoRushRed() {
    colourSortingSelector = 2;
    ringColour = false;
    chassis.setPose(-52.25, 24, -90);
    chassis.moveToPoint(-28, 24, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(5, 1500, {.minSpeed = 80, .earlyExitRange = 20});
    chassis.waitUntilDone();
    multitaskIntakeControls = true;
    chassis.moveToPose(-10, 41, 50, 1500, {.minSpeed =30});
    chassis.turnToHeading(0, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-10, 47, 1500);
    chassis.moveToPoint(-11, 36, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 8});
    chassis.moveToPoint(-24, 26, 2000, {.forwards = false});
    chassis.moveToPoint(-26, 46, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls = false;
    intake.move(0);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    intake.move(0);
    chassis.turnToHeading(80, 1500, {.direction = lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
    chassis.moveToPoint(-55, 32, 1500, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(-64.5, 7, 1500, {.forwards = false});
    chassis.swingToHeading(64, lemlib::DriveSide::LEFT, 1500, {.minSpeed = 80});
    chassis.turnToHeading(92, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls = true;
    lemlib:lemlib::Pose currentPose = chassis.getPose();
    chassis.moveToPoint(-65, currentPose.y, 500, {.forwards = false});
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
    chassis.setPose(-64, 0, currentPose.theta);
    pros::delay(800);
    chassis.moveToPose(-12, 22, 110, 1500);
    //chassis.moveToPose(-20, 22, 110, 1500);
    //chassis.turnToPoint(-24, 48, 1500, {.forwards = false});
}

void nonMogoRushBlue() {
    colourSortingSelector = 1;
    ringColour = true;
    chassis.setPose(52.25, 24, 90);
    chassis.moveToPoint(28, 24, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-5, 1500, {.minSpeed = 80, .earlyExitRange = 20});
    chassis.waitUntilDone();
    multitaskIntakeControls = true;
    chassis.moveToPose(10, 41, -50, 1500, {.minSpeed =30});
    chassis.turnToHeading(0, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(10, 47, 1500);
    chassis.moveToPoint(11, 36, 2000, {.forwards = false, .minSpeed = 60, .earlyExitRange = 8});
    chassis.moveToPoint(24, 26, 2000, {.forwards = false});
    chassis.moveToPoint(26, 46, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls = false;
    intake.move(0);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    intake.move(0);
    chassis.turnToHeading(-80, 1500, {.direction = lemlib::AngularDirection::CW_CLOCKWISE});
    chassis.moveToPoint(55, 32, 1500, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(64.5, 7, 1500, {.forwards = false});
    chassis.swingToHeading(-64, lemlib::DriveSide::RIGHT, 1500, {.minSpeed = 80});
    chassis.turnToHeading(-92, 1500);
    chassis.waitUntilDone();
    multitaskIntakeControls = true;
    lemlib:lemlib::Pose currentPose = chassis.getPose();
    chassis.moveToPoint(65, currentPose.y, 500, {.forwards = false});
    chassis.waitUntilDone();
    currentPose = chassis.getPose();
    chassis.setPose(64, 0, currentPose.theta);
    pros::delay(800);
    //chassis.moveToPose(8, 18, -110, 1500);
    chassis.moveToPose(11, 18, -110, 1500);
    chassis.turnToPoint(24, 48, 1500, {.forwards = false});
}

void mainMogoRushRed() {
    chassis.setPose(-52, -59.75, 75);
    wallStake.move(30);
    firstStageIntake.move(127);
	driveDistance(32.5, 127);
    int count = 0;
    while(LB_rotation.get_position() / 100 < 205) {
        wallStake.move(127);
        count += 1;
        if (count > 60) {
            break;
        }
        pros::delay(20);
    }
    wallStake.move(-60);
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
    wallStake.move(-30);
    chassis.turnToHeading(135, 1500);
    chassis.waitUntilDone();
    wallStake.move(-127);
    secondStageIntake.move(-50);
    chassis.moveToPoint(-24, -48, 1500, {.forwards = false});
    secondStageIntake.move(0);
    firstStageIntake.move(127);
    chassis.moveToPoint(-8, -31, 1000);
    chassis.moveToPoint(-21, -21, 1000, {.forwards = false});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    chassis.turnToHeading(-90, 1500);
    pros::delay(300);
    secondStageIntake.move(127);
    driveToDistance(8, 80, true);
    chassis.turnToHeading(-180, 1500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-63, -68, 1200);
    doinker.set_value(true);
    chassis.turnToHeading(90, 1500, {.minSpeed = 100});
}

void elimsMogoRushRed() { 
    mainMogoRushRed();
    wallStake.set_brake_mode(MOTOR_BRAKE_HOLD);
    while(LB_rotation.get_position() / 100 < 18) {
        wallStake.move(15);
    }
    wallStake.move(0);
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.moveToPoint(-15, -60, 1500);
    chassis.turnToHeading(180, 1500);
    driveToDistance(4, 60, false);
    chassis.turnToHeading(110, 1500);
    chassis.waitUntilDone();
    leftMotors.move(20);
    rightMotors.move(20);
    pros::delay(1000);
    chassis.swingToHeading(135, lemlib::DriveSide::LEFT, 1500);
    chassis.waitUntilDone();
    firstStageIntake.move(0);
    secondStageIntake.move(-10);
    wallStake.move(127);
}

void qualsMogoRushRed() {
    mainMogoRushRed();
    chassis.moveToPoint(-24, -20, 3000);
    chassis.waitUntilDone();
    wallStake.move(127);
    pros::delay(1500);
    wallStake.move(0);
}

void mainNegRed() {
    chassis.setPose(-63,13, 210);
    wallStake.move(127);
    pros::delay(750);
    wallStake.move(-4);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(200);
    chassis.moveToPose(-30, 24, 270, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40});
    chassis.moveToPose(-23, 24, 270, 1000, {.forwards = false, .maxSpeed = 50, .minSpeed = 40});
    wallStake.move(-127);
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(400);
    wallStake.move(0);
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x + 10  , currentPose.y + 0, currentPose.theta);
    chassis.moveToPose(-10, 34, 45, 2000);
    chassis.moveToPoint(-9, 54, 1500);
    chassis.waitUntilDone();
    if(secondStageIntake.get_actual_velocity() < 10) {
        secondStageIntake.move(-80);
        pros::delay(300);
        secondStageIntake.move(127);
    }
    chassis.moveToPoint(-22, 28, 1500, {.forwards = false});
    chassis.moveToPoint(-28, 47, 1500);
    chassis.turnToHeading(45, 1500);
    chassis.moveToPoint(-48, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    //driveToDistance(24, 100, true); // chagen back to true
    // chassis.turnToHeading(90, 1000);
    // chassis.waitUntilDone();
    
    chassis.turnToHeading(-90, 1500);
    if(secondStageIntake.get_actual_velocity() < 10) {
        secondStageIntake.move(-80);
        pros::delay(300);
        secondStageIntake.move(127);
    }
    chassis.waitUntilDone();
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    chassis.waitUntilDone();
    driveToDistance(8, 90, true);
    chassis.turnToHeading(-30, 1500, {.minSpeed = 20, .earlyExitRange = 10});
    chassis.moveToPoint(-72, 67, 1500, {.maxSpeed = 120, .minSpeed = 90});
    pros::delay(400);
}

void elimsNegRed() {
    mainNegRed();
    chassis.moveToPoint(-48, 48, 1500, {.forwards = false, .maxSpeed = 50});
    secondStageIntake.move(0);
    firstStageIntake.move(0);
    chassis.moveToPoint(-48, -24, 1500);
}

void qualsNegRed() {
    mainNegRed();
    chassis.moveToPoint(-24, 24, 1500, {.forwards = false, .maxSpeed = 50});
    chassis.moveToPoint(-20, 20, 1500);
    chassis.waitUntilDone();
    wallStake.move(127);
}

void sigAWPBlue() {
}