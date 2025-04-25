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
#include <future>

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
void driveD(double distance) {
    // double kP = 1;
    // double kD = 1;
    double inchToPosition = 56.2;
    double target = distance * inchToPosition + vertical_rotation.get_position() / 100.0;
    leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    double current = vertical_rotation.get_position() / 100.0;
    double error = target - current;
    // double prevError = error;
    // double deltaError;
    double Speed;
    if(distance > 0) {
        lemlib::PID pid(0.18, 0, 1);
        while (fabs(error) > 10 || fabs(current) < fabs(target) - 10) {
            current = vertical_rotation.get_position() / 100.0;
            error = target - current;
            Speed = pid.update(error) + 10;
            leftMotors.move(Speed);
            rightMotors.move(Speed);
            pros::delay(20);
        } 
    } else {
        lemlib::PID pid(0.12, 0, 0.7);
        while (fabs(error) > 10 || fabs(current) > fabs(target) + 10) {
            current = vertical_rotation.get_position() / 100.0;
            error = target - current;
            Speed = pid.update(error) - 10;
            if(Speed < -80) {
                Speed = -80;
            }
            leftMotors.move(Speed);
            rightMotors.move(Speed);
            pros::delay(20);
            std::cout << "target" << target << std::endl;
            std::cout << "current" << current << std::endl;
            std::cout << "error" << error << std::endl;
        } 
    }
    leftMotors.move(0);
    rightMotors.move(0);
    leftMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(MOTOR_BRAKE_BRAKE);  
}

void autoScoreAlliance() {
    double targetDistance = 10;
    double error;
    double kP = 10;
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    double currentDistance = distanceToWall.get_distance() / 25.4;
    while(currentDistance < targetDistance) {
        currentDistance = distanceToWall.get_distance() / 25.4;
        error = currentDistance - targetDistance;
        leftMotors.move(error * kP - 10);
        rightMotors.move(error * kP - 10);
    }
    leftMotors.move(0);
    rightMotors.move(0);
    currentState = 4;
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
    while(intake.get_actual_velocity() > 10 && counter < 145) {
        pros::delay(20);
        counter++;
        pros::screen::print(TEXT_LARGE_CENTER, 1, "velo: %f", intake.get_actual_velocity());
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
    if(intake.get_actual_velocity() < 10) {
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
	chassis.moveToPoint(-49, -22, 1500, {.forwards = false, .maxSpeed = 60});
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
    autoLoadLB();
    chassis.turnToHeading(180, 1000, {.minSpeed = 30});
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
    chassis.moveToPoint(-24, 24, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntil(6);
    intake.move(127);
    chassis.moveToPoint(-10, 36, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(22, 47, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000, {.minSpeed = 20});
    chassis.moveToPoint(2, 60, 1000);
    autoLoadLB();
    chassis.moveToPoint(2, 52, 1000, {.forwards = false});
    chassis.turnToHeading(0, 1000, {.minSpeed = 30});
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
    intake.move(0);
    chassis.moveToPoint(0, 46, 1500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.moveToPoint(24, 22, 1500);
    chassis.turnToHeading(120, 1500);
    chassis.waitUntilDone();
    checkJam();
    chassis.moveToPoint(48, 15, 1500, {.minSpeed = 40, .earlyExitRange = 3});
    chassis.waitUntilDone();
    intake.move(-127);
    chassis.moveToPoint(66, 55, 2500, {.minSpeed = 60});
    chassis.moveToPoint(57, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnToHeading(0, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    currentState = 1;
    resetPos();
    chassis.moveToPoint(52, 8, 1500, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    autoLoadLB();
    chassis.moveToPoint(48, -1, 1500, {.forwards = false, .maxSpeed = 50});
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
    driveToDistance(8, -60, false);
    currentState = 4;
    pros::delay(800);
    currentState = 3;
    leftMotors.move(-70);
    rightMotors.move(-70);	
    pros::delay(300);
	leftMotors.move(0);
    rightMotors.move(0);	
    currentState = 0;
    chassis.moveToPoint(24, 42, 2000, {.earlyExitRange = 5});
    chassis.waitUntilDone();
    intake.move(127);
    chassis.turnToPoint(48, 48, 1500, {.minSpeed = 25, .earlyExitRange = 5});
    chassis.moveToPoint(47, 47, 1500, {.minSpeed = 30});
    chassis.moveToPoint(40, 48, 1500, {.forwards = false, .minSpeed = 30});
    chassis.moveToPoint(48, 56, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(39, 54, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(180, 1000, {.minSpeed = 30, .earlyExitRange = 5});
    checkJam();
    chassis.moveToPoint(48, 0, 1500, {.minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToPoint(24, -24, 1500, {.minSpeed = 30});
    chassis.moveToPoint(24, -24, 2000);
    chassis.turnToHeading(180, 1500, {.minSpeed = 20});
    chassis.waitUntilDone();
    driveToDistance(12, 127, true);
    chassis.turnToHeading(90, 1500);
    chassis.waitUntilDone();
    driveToDistance(22, 127, true);
    chassis.turnToHeading(0, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(48, -47, 1500);
	chassis.waitUntilDone();
    chassis.moveToPoint(49, -57, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1});
	checkJam();
	chassis.waitUntilDone();
	chassis.moveToPoint(52, -57, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 1});
	checkJam();
    chassis.turnToHeading(-50, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
	pros::delay(300);
	intake.move(-127);
    leftMotors.move(-30);
    rightMotors.move(-40);
    pros::delay(700);
    leftMotors.move(0);
    rightMotors.move(0);

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

void scorePosBlue() {
    chassis.setPose(52, -59.75, -76);
    intake.move(127);
    currentState = 2;
	driveDistance(34, 127);
    int count = 0;
    currentState = 4;
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
}

void disruptPosBlue() {
    chassis.setPose(52, -59.75, -76);
    intake.move(127);
    currentState = 4;
	driveDistance(34, 127);
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
}


// void mainPosBlue() {
//     chassis.turnToHeading(-140, 1500);
//     chassis.waitUntilDone();
//     currentState = 2;
//     intake.move(-50);
//     chassis.moveToPoint(24, -48, 1500, {.forwards = false});
//     intake.move(0);
//     intake.move(127);
//     chassis.moveToPoint(8, -40, 1000);
//     chassis.moveToPoint(22, -22, 1000, {.forwards = false});
//     chassis.moveToPoint(23, -26, 1000, {.forwards = false, .maxSpeed = 30});
//     chassis.waitUntilDone();
//     mogoMech.set_value(true);
//     pros::delay(200); 
//     chassis.moveToPoint(44, -65, 1500);
//     chassis.turnToHeading(110, 1000);
//     chassis.waitUntilDone();
//     doinker.set_value(true);
//     intake.move(127);
//     chassis.moveToPoint(63, -67, 1500, {.maxSpeed = 50, .minSpeed = 49});
//     chassis.turnToHeading(0, 1500);
// }

void mainPosBlue() {
    chassis.setPose(58, -37.25, -90);
    intake.move(127);
    chassis.moveToPoint(21, -45, 1500, {.minSpeed = 90, .earlyExitRange = 3});
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.moveToPoint(18, -47, 1500, {.minSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(32, -48, 1500, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(300);
    chassis.turnToPoint(14, -40, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 40, .earlyExitRange = 5});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    intake.move(127);
    pros::delay(500);
    intake.move(0);
    chassis.moveToPoint(28, -40, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1500, {.maxSpeed = 80, .minSpeed = 30});
    chassis.waitUntilDone();
    driveToDistance(18, 80, true);
    chassis.turnToHeading(-150, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    chassis.turnToPoint(44, -24, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(48, -27, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(90, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(28, -27, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    intake.move(127);
    chassis.moveToPoint(38, -65, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.turnToHeading(110, 1000, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(72, -71, 1250, {.maxSpeed = 60, .minSpeed = 59});
    chassis.turnToHeading(0, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    mogoState = true;
}

void elimsPosBlue() {
    mainPosBlue();
    //autoLoadLB();
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.turnToHeading(-90, 1500);
    chassis.moveToPoint(32, -48, 2000);
    // chassis.moveToPoint(15, -64, 1500, {.maxSpeed = 70});
    // chassis.swingToHeading(-130, lemlib::DriveSide::LEFT, 1500);
    // chassis.waitUntilDone();
    // currentState = 2;
}

void qualsPosBlue() {
    mainPosBlue();
    chassis.moveToPoint(30, -28, 3000);
    chassis.waitUntil(25);
    currentState = 4;
}

void scorePosRed() {
    chassis.setPose(-52, -59.75, 76);
    intake.move(127);
    currentState = 2;
	driveDistance(34, 127);
    int count = 0;
    currentState = 4;
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
    pros::delay(350);
}

void disruptPosRed() {
    chassis.setPose(-52, -59.75, 76);
    intake.move(127);
    currentState = 4;
	driveDistance(34, 127);
    int count = 0;
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 4, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 7, "theta: %f", pose.theta);
}

// void mainPosRed() {
//     chassis.turnToHeading(140, 1500);
//     chassis.waitUntilDone();
//     currentState = 2;
//     intake.move(-50);
//     chassis.moveToPoint(-33, -48, 1500, {.forwards = false});
//     intake.move(0);
//     intake.move(127);
//     chassis.moveToPoint(-10, -36, 1000);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-22, -22, 1000, {.forwards = false, .maxSpeed = 70});
//     chassis.moveToPoint(-27, -28, 1000, {.forwards = false, .maxSpeed = 30});
//     chassis.waitUntilDone();
//     mogoMech.set_value(true);
//     pros::delay(300);
//     chassis.turnToHeading(-90, 1500);
//     pros::delay(300);
//     intake.move(127);
//     driveToDistance(8, 80, true);
//     checkJam();
//     chassis.turnToHeading(-150, 1500);
//     chassis.waitUntilDone();
//     chassis.moveToPoint(-67, -68, 1200, {.maxSpeed = 50, .minSpeed = 49});
//     doinker.set_value(true);
//     chassis.turnToHeading(90, 1500, {.minSpeed = 100});
// }

void mainPosRed() {
    chassis.setPose(-58, -58.75, 90);
    intake.move(127);
    chassis.moveToPoint(-21, -45, 1500, {.minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.moveToPoint(-18, -47, 1500, {.minSpeed = 50});
    chassis.waitUntilDone();
    chassis.moveToPoint(-32, -48, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(300);
    chassis.turnToPoint(-14, -45, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 5});
    chassis.moveToPoint(-16, -45, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    intake.move(127);
    pros::delay(500);
    intake.move(0);
    chassis.moveToPoint(-28, -37, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1500, {.maxSpeed = 80, .minSpeed = 30});
    chassis.waitUntilDone();
    driveToDistance(18, 80, true);
    chassis.moveToPoint(-48, -16, 1500, {.forwards = false, .minSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    chassis.moveToPoint(-48, -24, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-26, -19, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 30});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    intake.move(0);
    chassis.turnToHeading(-90, 1500);
    driveToDistance(8, 80, true);
    intake.move(127);
    chassis.turnToHeading(-150, 1500);
    chassis.waitUntilDone();
    chassis.moveToPoint(-70, -74, 1000, {.maxSpeed = 75, .minSpeed = 60});
    doinker.set_value(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000, {.minSpeed = 100});
}

void elimsPosRed() { 
    // disruptPosRed();
    mainPosRed();
    // autoLoadLB();
    // chassis.waitUntilDone();
    // doinker.set_value(false);
    // chassis.turnToHeading(90, 1500);
    // chassis.moveToPose(-11, -64, 145, 1500, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // currentState = 2;
    chassis.waitUntilDone();
    doinker.set_value(false);
    // pros::delay(1500);
    // chassis.moveToPoint(-44, -60, 1500);
    chassis.moveToPoint(-32, -48, 1000);
    mogoState = true;
    chassis.waitUntilDone();
}

void qualsPosRed() {
    // scorePosRed();
    mainPosRed();
    // autoLoadLB();
    // chassis.waitUntilDone();
    // doinker.set_value(false);
    // chassis.turnToHeading(90, 1500);
    // chassis.moveToPose(-11, -64, 145, 1500, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // currentState = 2;
    doinker.set_value(false);
    // pros::delay(1500);
    // chassis.moveToPoint(-44, -60, 1500);
    chassis.moveToPoint(-21, -26, 3000);
    chassis.waitUntilDone();
    currentState = 4;
    mogoState = true;
}

/*void mainNegRed() {
    chassis.setPose(-63,13, 210);
    currentState = 4;
    pros::delay(650);
    currentState = 0;
    pros::delay(500);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(200);
    chassis.moveToPose(-30, 21, 270, 1000, {.forwards = false, .minSpeed = 70});
    chassis.waitUntilDone();
    chassis.moveToPoint(-23, 20, 1000, {.forwards = false, .minSpeed = 20});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(100);
    intake.move(127);
    intake.move(127);
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    checkJam();
    chassis.moveToPose(-11.5, 34, 45, 2000, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.moveToPoint(-10.5, 54, 1500, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.waitUntilDone();
    checkJam();
    chassis.moveToPoint(-22, 28, 1500, {.forwards = false, .minSpeed = 50, .earlyExitRange = 5});
    chassis.moveToPoint(-27, 47, 1500);
    chassis.turnToHeading(45, 1500);
    chassis.moveToPoint(-48, 44, 1500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1500);
    checkJam();
    chassis.waitUntilDone();
    intake.move(127);
    intake.move(127);
    chassis.waitUntilDone();
    driveToDistance(10, 90, true);
    chassis.turnToHeading(-10, 1000, {.minSpeed = 20, .earlyExitRange = 10});
    chassis.moveToPoint(-68, 66, 1500, {.minSpeed = 120});
    chassis.moveToPoint(-75, 65, 600, {.minSpeed = 120}); 
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 47, 1500, {.forwards = false, .maxSpeed = 70, .minSpeed = 50, .earlyExitRange = 5});
}*/

void mainNegRed() {
    chassis.setPose(-53.5, 64, 90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    doinker.set_value(true);
    intakeSpeed = 127;    
    chassis.moveToPose(-6, 52, 120, 1500, {.lead = 0.3, .minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(250);
    intakeSpeed = 127;
    chassis.moveToPoint(-32, 61, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(11);
    intakeSpeed = 0;
    chassis.swingToHeading(30, lemlib::DriveSide::RIGHT, 500, {.minSpeed = 30, .earlyExitRange=5});
    chassis.moveToPoint(-34, 38, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    intakeSpeed = 0;

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x - 3, currentPose.y + 3, currentPose.theta);

    chassis.moveToPoint(-24, 24, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(0,300);
    chassis.waitUntilDone();
    intakeSpeed = 127;
    chassis.moveToPoint(-23, 46, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(-46, 48, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-58, 55, 1500, {.maxSpeed = 90, .minSpeed = 30});
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToDistance(6, 100, true);
    chassis.turnToHeading(-20, 300);
    chassis.waitUntilDone();
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(200);
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(700);
    intakeSpeed = 0;
    doubleLBLoading = true;
    leftMotors.move(-80);
    rightMotors.move(-80);
    pros::delay(400);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(700);
    chassis.turnToHeading(0, 300, {.minSpeed = 30});
    chassis.waitUntilDone();
    doubleLBLoading = false;
    intakeSpeed = 127;
    resetPos();
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intakeSpeed = 0;
    currentState = 6;
    chassis.turnToPoint(-72, 0, 500);
    chassis.waitUntilDone();
    intakeSpeed = 127;
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(600);
    leftMotors.move(0);
    rightMotors.move(0);
    autoScoreAlliance();
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
    leftMotors.move(-70);
    rightMotors.move(-70);
    pros::delay(200);
    currentState = 0;
    chassis.turnToHeading(90, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    leftMotors.move(60);
    rightMotors.move(60);
}

void mainNegBlue() {
    chassis.setPose(53.5, 64, 90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    doinker.set_value(true);
    intakeSpeed = 127;    
    chassis.moveToPose(-6, 48, -60, 1500, {.lead = 0.3, .minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(250);
    intakeSpeed = 127;

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x - 0, currentPose.y + 0, currentPose.theta);

    chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntil(11);
    intakeSpeed = 0;
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-20,300);
    chassis.waitUntilDone();
    doinker.set_value(false);
    intakeSpeed = 127;
    chassis.moveToPoint(24, 46, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(46, 48, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(0, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    driveToDistance(6, 100, true);
    chassis.turnToHeading(70, 300);
    chassis.waitUntilDone();
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(200);
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(700);
    intakeSpeed = 0;
    doubleLBLoading = true;
    leftMotors.move(-80);
    rightMotors.move(-80);
    pros::delay(400);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(700);
    chassis.waitUntilDone();
    doubleLBLoading = false;
    intakeSpeed = 127;
    chassis.moveToPoint(48, 24, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intakeSpeed = 0;
    currentState = 6;
    chassis.turnToPoint(72, 0, 500);
    chassis.waitUntilDone();
    intakeSpeed = 127;
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(600);
    leftMotors.move(0);
    rightMotors.move(0);
    autoScoreAlliance();
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
    chassis.moveToPoint(19, 22, 1500);
    chassis.waitUntil(25);
    currentState = 4;
    chassis.turnToHeading(-135, 1500);
}