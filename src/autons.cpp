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
    while(fabs(disToWall - distance) > 0.5) {
        disToWall = distanceToWall.get_distance() / 25.4;
        error = disToWall - distance;
        if(fabs(error*kP) > abs(maxSpeed)) {
            leftMotors.move(maxSpeed);
            rightMotors.move(maxSpeed);
        } else {
            if(error > 0) {
                leftMotors.move(error * kP + 20);
                rightMotors.move(error * kP + 20);
            } else {
                leftMotors.move(error * kP - 20);
                rightMotors.move(error * kP - 20);
            }
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
            chassis.setPose(currentPose.x, 67 - disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta - 90) < 8) {
            chassis.setPose(67 - disToWall, currentPose.y, currentPose.theta);
        } else if(fabs(absoluteTheta - 180) < 8 || fabs(currentPose.theta + 180) < 8) {
            chassis.setPose(currentPose.x, -67 + disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta + 90) < 8) {
            chassis.setPose(-67 + disToWall, currentPose.y, currentPose.theta);
        }
    }
}

void resetPos() {
    double disToWall = distanceToWall.get_distance() / 25.4;
    lemlib::Pose currentPose = chassis.getPose();
        double absoluteTheta = (int)currentPose.theta % 360;
        if(fabs(absoluteTheta - 0) < 8) {
            chassis.setPose(currentPose.x, 67 - disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta - 90) < 8) {
            chassis.setPose(67 - disToWall, currentPose.y, currentPose.theta);
        } else if(fabs(absoluteTheta - 180) < 8 || fabs(currentPose.theta + 180) < 8) {
            chassis.setPose(currentPose.x, -67 + disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta + 90) < 8) {
            chassis.setPose(-67 + disToWall, currentPose.y, currentPose.theta);
        }
}

void driveDistance(double distance, double maxSpeed) {
    double kP = 4;
    double error;
    double degreeToInches = 0.01854411111;
    leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    double startingPos = vertical_rotation.get_position() / 100.0;
    double destinationPos = startingPos * degreeToInches + distance;
    double motorSpeed;
    if(distance > 0) {
        while(vertical_rotation.get_position() / 100.0 * degreeToInches < destinationPos - 1) {
            error = destinationPos - (vertical_rotation.get_position() / 100.0 * degreeToInches);
            if(error * kP + 15 > maxSpeed) {
                motorSpeed = maxSpeed; 
            } else {
                motorSpeed = error * kP + 15;
            }
            if(motorSpeed > (vertical_rotation.get_position() / 200.0 - startingPos)) {
                motorSpeed = (vertical_rotation.get_position() / 200.0 - startingPos) + 50;
            }
            leftMotors.move(motorSpeed);
            rightMotors.move(motorSpeed);
            pros::delay(20);
            pros::screen::print(TEXT_LARGE_CENTER, 1, "currentPos: %f\n", rightMotors.get_position()* degreeToInches);
            //pros::screen::print(TEXT_LARGE_CENTER, 10, "destinationPos: %d", destinationPos);
        }
        leftMotors.move(0);
        rightMotors.move(0);
        leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
        rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);  
    } else {
       while(vertical_rotation.get_position() * degreeToInches < destinationPos) {
            error = destinationPos - vertical_rotation.get_position() * degreeToInches;
            if(error * kP + 30 > maxSpeed) {
                motorSpeed = maxSpeed; 
            } else {
                motorSpeed = error * kP + 30;
            }
            if(motorSpeed > (vertical_rotation.get_position() - startingPos) * 360) {
                motorSpeed = (vertical_rotation.get_position() - startingPos) * 360 + 50;
            }
            leftMotors.move(motorSpeed);
            rightMotors.move(motorSpeed);
            pros::delay(20);
            pros::screen::print(TEXT_LARGE_CENTER, 1, "currentPos: %f\n", rightMotors.get_position()* degreeToInches);
            //pros::screen::print(TEXT_LARGE_CENTER, 10, "destinationPos: %d", destinationPos);
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

void autoLoadLB() {
    currentState = 1;
    int counter = 0;
    intake.move(127);
    pros::delay(300);
    while(secondStageIntake.get_actual_velocity() > 10 && counter < 150) {
        pros::delay(20);
        counter++;
        pros::screen::print(TEXT_LARGE_CENTER, 1, "velo: %f", secondStageIntake.get_actual_velocity());
    }
    intake.move(0);
}

void autoScoreLB() {
    currentState = 2;
    pros::delay(600);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(200);
    leftMotors.move(0);
    rightMotors.move(0);
    currentState = 0;
}

void checkJam() {
    if(secondStageIntake.get_actual_velocity() < 10) {
        intake.move(-127);
        pros::delay(200);
        intake.move(127);
    }
}
void skillsAuton() {
    chassis.setPose(-61, 0, 90); 
    intake.move(127);
	pros::delay(300);   
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(250);
    intake.move(0);
	chassis.turnToHeading(-30, 1200, {.minSpeed = 30, .earlyExitRange = 10});
	chassis.moveToPoint(-49, -22, 1500, {.forwards = false, .maxSpeed = 90});
	chassis.waitUntil(30);
	mogoMech.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-24, -26, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntil(6);
    intake.move(127);
    chassis.moveToPoint(-10, -36, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(22, -49, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 20});
    chassis.moveToPoint(-1, -60, 1000);
    chassis.turnToHeading(180, 1000, {.minSpeed = 30});
    autoLoadLB();
    chassis.waitUntilDone();
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(600);
    leftMotors.move(-5);
    rightMotors.move(-5);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, -63, currentPose.theta);
    autoScoreLB();
    intake.move(-127);
    //SCORED THE FIRST WALLSTAKE!! and scored the first two rings on mogooo
    chassis.moveToPoint(0, -48, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    checkJam();
    currentState = 0;
    chassis.turnToHeading(-90, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    intake.move(127);
    chassis.moveToPoint(-24, -48, 1500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToHeading(-90, 1500);
    driveToDistance(8, 60, true);
    leftMotors.move(-70);
    rightMotors.move(-70);
    pros::delay(400);
    chassis.moveToPoint(-48, -60, 3000, {.earlyExitRange=10});
    checkJam();
    chassis.waitUntilDone();
    chassis.moveToPoint(-59, -62, 3000, {.forwards=false, .minSpeed=30});
    checkJam();
    chassis.waitUntilDone();
    pros::delay(300);
    mogoMech.set_value(false);
    intake.move(-127);
    pros::delay(150);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x + 2, currentPose.y, currentPose.theta);
    chassis.follow(skillsPath4_txt, 18, 2000);
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    driveToDistance(17, 80, true);
    pros::delay(300);
    chassis.turnToHeading(180, 1500, {.minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-48, 24, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(0);
    mogoMech.set_value(true);
    pros::delay(150);
    chassis.moveToPoint(-24, 26, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntil(6);
    intake.move(127);
    chassis.moveToPoint(-10, 36, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(22, 49, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 20});
    chassis.moveToPoint(2, 60, 1000);
    chassis.turnToHeading(0, 1000, {.minSpeed = 30});
    autoLoadLB();
    chassis.waitUntilDone();
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(600);
    leftMotors.move(-5);
    rightMotors.move(-5);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, 63, currentPose.theta);
    autoScoreLB();
    intake.move(-127);

    //SCORED THE SECOND WALLSTAKE!!    
    
    chassis.moveToPoint(0, 48, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    checkJam();
    currentState = 0;
    chassis.turnToHeading(-60, 1500, {.minSpeed = 80, .earlyExitRange = 5});
    intake.move(127);
    chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToHeading(-90, 1500);
    driveToDistance(8, 60, true);
    leftMotors.move(-70);
    rightMotors.move(-70);
    pros::delay(400);
    chassis.moveToPoint(-48, 60, 3000, {.earlyExitRange=10});
    checkJam();
    chassis.waitUntilDone();
    chassis.moveToPoint(-59, 62, 3000, {.forwards=false, .minSpeed=30});
    checkJam();
    chassis.waitUntilDone();
    pros::delay(500);
    mogoMech.set_value(false);
    intake.move(-127);
    pros::delay(200);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    skillsAutonSecondHalf();
}

void skillsAutonSecondHalf() {
    //chassis.follow(skillsPath7_txt, 15, 6000);
    chassis.moveToPoint(0, 48, 1500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    secondStageIntake.move(45);
    firstStageIntake.move(127);
    chassis.moveToPoint(24, 22, 1500);
    chassis.turnToHeading(120, 1500);
    chassis.waitUntilDone();
    chassis.moveToPoint(48, 15, 1500, {.minSpeed = 40, .earlyExitRange = 3});
    chassis.waitUntilDone();
    secondStageIntake.move(0);
    firstStageIntake.move(-127);
    chassis.moveToPoint(66, 55, 2500, {.minSpeed = 60});
    chassis.moveToPoint(57, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    firstStageIntake.move(127);
    chassis.turnToHeading(0, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    autoLoadLB();
    resetPos();
    chassis.moveToPoint(52, 8, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPoint(48, 0, 1500, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntil(4);
    mogoMech.set_value(true);
    pros::delay(300);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    chassis.turnToHeading(90, 1500, {.minSpeed = 10});
    chassis.waitUntilDone();
    leftMotors.move(90);
    rightMotors.move(90);
    pros::delay(800);
    leftMotors.move(0);
    rightMotors.move(0);
    driveToDistance(8, -60, true);
    currentState = 4;
    pros::delay(800);
    currentState = 3;
    driveToDistance(14, -60, false);
    currentState = 0;
    chassis.moveToPoint(24, 42, 2000, {.earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnToPoint(48, 48, 1500, {.minSpeed = 25, .earlyExitRange = 5});
    chassis.moveToPoint(48, 48, 1500, {.minSpeed = 30});
    chassis.moveToPoint(38, 48, 1500, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(45, 58, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(39, 54, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(180, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    checkJam();
    chassis.moveToPoint(48, 0, 1500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToPoint(24, -24, 1500, {.minSpeed = 30});
    chassis.moveToPoint(24, -24, 2000);
    chassis.turnToHeading(180, 1500, {.minSpeed = 20});
    chassis.waitUntilDone();
    driveToDistance(15, 127, true);
    chassis.turnToHeading(90, 1500);
    chassis.waitUntilDone();
    driveToDistance(22, 127, true);
    chassis.turnToHeading(0, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(48, -50, 1500);
    chassis.turnToHeading(150, 1500);
    chassis.waitUntilDone();
    secondStageIntake.move(0);
    chassis.moveToPoint(58, -58, 1500, {.minSpeed = 35, .earlyExitRange = 5});
    chassis.turnToHeading(-20, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    secondStageIntake.move(-127);
    pros::delay(100);
    leftMotors.move(-40);
    rightMotors.move(-40);
    // chassis.turnToHeading(90, 1500, {.minSpeed = 50});
    // pros::delay(500);
    // chassis.moveToPoint(36, -36, 2000, {.minSpeed = 30, .earlyExitRange = 10});
    // chassis.moveToPoint(0, 0, 3000, {.forwards = false,.maxSpeed = 60, .minSpeed = 30});
    // while(wallStake.get_position() < 1500) {
    //     wallStake.move(80);
    // }
    // wallStake.move(0);
    // pros::delay(5000);
    // doinker.set_value(true);
}


void mainPosBlue() {
    chassis.setPose(52, -59.75, -76);
    firstStageIntake.move(127);
    currentState = 2;
	driveDistance(34, 127);
    int count = 0;
    currentState = 4;
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
    chassis.turnToHeading(-135, 1500);
    chassis.waitUntilDone();
    currentState = 0;
    secondStageIntake.move(-50);
    chassis.moveToPoint(24, -48, 1500, {.forwards = false});
    secondStageIntake.move(0);
    firstStageIntake.move(127);
    chassis.moveToPoint(8, -40, 1000);
    chassis.moveToPoint(22, -22, 1000, {.forwards = false});
    chassis.moveToPoint(23, -26, 1000, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200); 
    chassis.moveToPoint(44, -65, 1500);
    chassis.turnToHeading(110, 1000);
    chassis.waitUntilDone();
    doinker.set_value(true);
    secondStageIntake.move(127);
    chassis.moveToPoint(63, -67, 1500);
    chassis.turnToHeading(0, 1500);
}

void elimsPosBlue() {
    mainPosBlue();
    autoLoadLB();
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.turnToHeading(-90, 1500);
    chassis.moveToPoint(15, -64, 1500, {.maxSpeed = 70});
    chassis.swingToHeading(-130, lemlib::DriveSide::LEFT, 1500);
    chassis.waitUntilDone();
    currentState = 2;
}

void qualsPosBlue() {
    
}

void mainPosRed() {
    chassis.setPose(-52, -59.75, 76);
    firstStageIntake.move(127);
    currentState = 2;
	driveDistance(34, 127);
    int count = 0;
    currentState = 4;
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
    chassis.turnToHeading(135, 1500);
    chassis.waitUntilDone();
    currentState = 0;
    secondStageIntake.move(-50);
    chassis.moveToPoint(-24, -48, 1500, {.forwards = false});
    secondStageIntake.move(0);
    firstStageIntake.move(127);
    chassis.moveToPoint(-8, -40, 1000);
    chassis.moveToPoint(-22, -22, 1000, {.forwards = false});
    chassis.moveToPoint(-23, -26, 1000, {.forwards = false, .maxSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(300);
    chassis.turnToHeading(-90, 1500);
    pros::delay(300);
    secondStageIntake.move(127);
    driveToDistance(8, 80, true);
    chassis.turnToHeading(180, 1500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-63, -68, 1200);
    doinker.set_value(true);
    chassis.turnToHeading(90, 1500, {.minSpeed = 100});
}

void elimsPosRed() { 
    mainPosRed();
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

void qualsPosRed() {
    mainPosRed();
    chassis.moveToPoint(-24, -20, 3000);
    chassis.waitUntilDone();
    wallStake.move(127);
    pros::delay(1500);
    wallStake.move(0);
}

void mainNegRed() {
    chassis.setPose(-63,13, 210);
    currentState = 4;
    pros::delay(650);
    currentState = 3;
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(200);
    chassis.moveToPose(-30, 23, 270, 1000, {.forwards = false, .minSpeed = 70});
    chassis.waitUntilDone();
    currentState = 0;
    chassis.moveToPoint(-23, 24, 1000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(100);
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    checkJam();
    chassis.moveToPose(-10, 34, 45, 2000, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.moveToPoint(-9, 54, 1500, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.waitUntilDone();
    checkJam();
    chassis.moveToPoint(-22, 28, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(-28, 47, 1500);
    chassis.turnToHeading(45, 1500);
    chassis.moveToPoint(-48, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1500);
    checkJam();
    chassis.waitUntilDone();
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    chassis.waitUntilDone();
    driveToDistance(8, 90, true);
    chassis.turnToHeading(-30, 1500, {.minSpeed = 20, .earlyExitRange = 10});
    chassis.moveToPoint(-72, 67, 1500, {.maxSpeed = 120, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveToPoint(-48, 48, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 5});
}

void elimsNegRed() {
    mainNegRed();
    chassis.turnToHeading(180, 1500);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(-48, -24, 1500);
}

void qualsNegRed() {
    mainNegRed();
    chassis.moveToPoint(-21, 21, 1500);
    chassis.waitUntil(25);
    currentState = 3;
    chassis.turnToHeading(135, 1500);
}

void mainNegBlue() {
    chassis.setPose(63,13, -210);
    currentState = 4;
    pros::delay(650);
    currentState = 3;
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(200);
    chassis.moveToPose(30, 23, -270, 1000, {.forwards = false, .minSpeed = 70});
    chassis.waitUntilDone();
    currentState = 0;
    chassis.moveToPoint(23, 24, 1000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(100);
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    checkJam();
    chassis.moveToPose(10, 34, -45, 2000, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.moveToPoint(9, 54, 1500, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.waitUntilDone();
    checkJam();
    chassis.moveToPoint(22, 28, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(28, 47, 1500);
    chassis.turnToHeading(-45, 1500);
    chassis.moveToPoint(48, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1500);
    checkJam();
    chassis.waitUntilDone();
    firstStageIntake.move(127);
    secondStageIntake.move(127);
    chassis.waitUntilDone();
    driveToDistance(8, 90, true);
    chassis.turnToHeading(30, 1500, {.minSpeed = 20, .earlyExitRange = 10});
    chassis.moveToPoint(72, 67, 1500, {.maxSpeed = 120, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(600);
    chassis.moveToPoint(48, 48, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 5});
}

void elimsNegBlue() {
    mainNegBlue();
    chassis.turnToHeading(-180, 1500);
    chassis.waitUntilDone();
    intake.move(0);
    chassis.moveToPoint(48, -24, 1500);
}

void qualsNegBlue() {
    mainNegBlue();
    chassis.moveToPoint(21, 21, 1500);
    chassis.waitUntil(25);
    currentState = 3;
    chassis.turnToHeading(-135, 1500);
}