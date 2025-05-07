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
    int timeoutCounter = 0;
    while(fabs(disToWall - distance) > 0.5 && timeoutCounter < 150) {
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
        timeoutCounter++;
        pros::screen::print(TEXT_LARGE_CENTER, 7, "distance: %i\n", distanceToWall.get());
        pros::delay(20);
        pros::screen::erase();
    }
    leftMotors.move(0);
    rightMotors.move(0);
    if(resetPos) {
        lemlib::Pose currentPose = chassis.getPose();
        double absoluteTheta = (int)currentPose.theta % 360;
        if(absoluteTheta > 180) {
            absoluteTheta -= 360;
        }
        if(fabs(absoluteTheta - 0) < 8) {
            chassis.setPose(currentPose.x, 70 - disToWall - 4, currentPose.theta);
        } else if(fabs(absoluteTheta - 90) < 8) {
            chassis.setPose(70 - disToWall - 4, currentPose.y, currentPose.theta);
        } else if(fabs(absoluteTheta - 180) < 8 || fabs(currentPose.theta + 180) < 8) {
            chassis.setPose(currentPose.x, -70 + disToWall + 4, currentPose.theta);
        } else if(fabs(absoluteTheta + 90) < 8) {
            chassis.setPose(-70 + disToWall + 4, currentPose.y, currentPose.theta);
        }
    }
}

void driveToAxisPos(char axis, int destinationValue, bool direction, int intentedAngle, int maxSpeed) {
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    double currentValue, startingTheta, thetaCorrection, thetaCorrectSpeedLimit = 30;

    lemlib::Pose currentPose = chassis.getPose();
    startingTheta = intentedAngle;

    double error, thetaDeviation, kP = 5, kT = 1.5;
    double kD = 0.6; // Derivative gain
    // kP refers to linear motion, kT refers to robot drift correction, kD is derivative
    double leftMotorsPower, rightMotorsPower;
    double lastError = 0; // For derivative calculation
    double derivative = 0;

    int timeoutCounter = 0, timeoutThreshold = 100;

    if(direction) {
        if(axis == 'x') {
            currentPose = chassis.getPose();
            lastError = fabs(destinationValue - currentPose.x);
            while(fabs(destinationValue - currentPose.x) > 1 && timeoutCounter < timeoutThreshold) {
                timeoutCounter++;
                currentPose = chassis.getPose();
                double currentTheta = (int)currentPose.theta % 360;
                if(currentTheta > 180) {
                    currentTheta -= 360;
                }

                // linear PD code
                error = fabs(destinationValue - currentPose.x);
                derivative = (error - lastError) / 0.02; // 20ms loop
                lastError = error;
                double pdTerm = error * kP + kD * derivative;
                leftMotorsPower = pdTerm + 20;
                rightMotorsPower = pdTerm + 20;
                
                // // drift correction code
                // thetaDeviation = currentTheta - startingTheta;
                // thetaCorrection = thetaDeviation * kT;
                // if(fabs(thetaCorrection) > thetaCorrectSpeedLimit) {
                //     if(thetaCorrection > 0) thetaCorrection = thetaCorrectSpeedLimit;
                //     else thetaCorrection = -thetaCorrectSpeedLimit;
                // }

                // leftMotorsPower -= thetaCorrection;
                // rightMotorsPower += thetaCorrection;

                if(leftMotorsPower > maxSpeed) {
                    leftMotorsPower = maxSpeed;
                    rightMotorsPower = maxSpeed;
                }

                leftMotors.move(leftMotorsPower);
                rightMotors.move(rightMotorsPower);

                pros::delay(20);
            }
        } else if (axis == 'y'){
            currentPose = chassis.getPose();
            lastError = fabs(destinationValue - currentPose.y);
            while(fabs(destinationValue - currentPose.y) > 1 && timeoutCounter < timeoutThreshold) {
                timeoutCounter++;
                currentPose = chassis.getPose();
                double currentTheta = (int)currentPose.theta % 360;
                if(currentTheta > 180) {
                    currentTheta -= 360;
                }

                // linear PD code
                error = fabs(destinationValue - currentPose.y);
                derivative = (error - lastError) / 0.02;
                lastError = error;
                double pdTerm = error * kP + kD * derivative;
                leftMotorsPower = pdTerm + 20;
                rightMotorsPower = pdTerm + 20;
                
                // // drift correction code
                // thetaDeviation = currentTheta - startingTheta;

                // leftMotorsPower -= thetaDeviation * kT;
                // rightMotorsPower += thetaDeviation * kT;

                if(leftMotorsPower > maxSpeed) {
                    leftMotorsPower = maxSpeed;
                    rightMotorsPower = maxSpeed;
                }

                leftMotors.move(leftMotorsPower);
                rightMotors.move(rightMotorsPower);

                pros::delay(20);
            }
        } else {
            controller.rumble("...");
        }
    } else {
        if(axis == 'x') {
            currentPose = chassis.getPose();
            lastError = -1 * fabs(destinationValue - currentPose.x);
            while(fabs(destinationValue - currentPose.x) > 1 && timeoutCounter < timeoutThreshold) {
                timeoutCounter++;
                currentPose = chassis.getPose();
                double currentTheta = (int)currentPose.theta % 360;
                if(currentTheta > 180) {
                    currentTheta -= 360;
                }

                // linear PD code
                error = -1 * fabs(destinationValue - currentPose.x);
                derivative = (error - lastError) / 0.02;
                lastError = error;
                double pdTerm = error * kP + kD * derivative;
                leftMotorsPower = pdTerm - 20;
                rightMotorsPower = pdTerm - 20;
                
                // // drift correction code
                // thetaDeviation = currentTheta - startingTheta;
                // thetaCorrection = thetaDeviation * kT;
                // if(fabs(thetaCorrection) > 15) {
                //     if(thetaCorrection > 0) thetaCorrection = 15;
                //     else thetaCorrection = -15;
                // }

                // leftMotorsPower -= thetaCorrection;
                // rightMotorsPower += thetaCorrection;

                if(leftMotorsPower < maxSpeed) {
                    leftMotorsPower = maxSpeed;
                    rightMotorsPower = maxSpeed;
                }

                leftMotors.move(leftMotorsPower);
                rightMotors.move(rightMotorsPower);

                pros::delay(20);
            }
        } else if (axis == 'y'){
            currentPose = chassis.getPose();
            lastError = -1 * fabs(destinationValue - currentPose.y);
            while(fabs(destinationValue - currentPose.y) > 1 && timeoutCounter < timeoutThreshold) {
                timeoutCounter++;
                currentPose = chassis.getPose();
                double currentTheta = (int)currentPose.theta % 360;
                if(currentTheta > 180) {
                    currentTheta -= 360;
                }

                // linear PD code
                error = -1 * fabs(destinationValue - currentPose.y);
                derivative = (error - lastError) / 0.02;
                lastError = error;
                double pdTerm = error * kP + kD * derivative;
                leftMotorsPower = pdTerm - 20;
                rightMotorsPower = pdTerm - 20;
                
                // // drift correction code
                // thetaDeviation = currentTheta - startingTheta;

                // leftMotorsPower -= thetaDeviation * kT;
                // rightMotorsPower += thetaDeviation * kT;

                if(leftMotorsPower < maxSpeed) {
                    leftMotorsPower = maxSpeed;
                    rightMotorsPower = maxSpeed;
                }

                leftMotors.move(leftMotorsPower);
                rightMotors.move(rightMotorsPower);

                pros::delay(20);
            }
        } else {
            controller.rumble("...");
        }
    }
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    leftMotors.move(0);
    rightMotors.move(0);
}

void resetPos() {
    lemlib::Pose currentPose = chassis.getPose();

    int numberOfReadings = 1;
    double totalSumOfReadings = 0;

    for(int i = 0; i < numberOfReadings; i++) {
        totalSumOfReadings += distanceToWall.get_distance() / 25.4;
    }

    double disToWall = totalSumOfReadings / numberOfReadings;


        double absoluteTheta = (int)currentPose.theta % 360;
        if(absoluteTheta > 180) {
            absoluteTheta -= 360;
        }
        if(absoluteTheta < -180) {
            absoluteTheta += 360;
        }
        std::cout <<absoluteTheta <<"\n";
        if(fabs(absoluteTheta - 0) < 10) {
            chassis.setPose(currentPose.x, 65 - disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta - 90) < 10) {
            chassis.setPose(65 - disToWall, currentPose.y, currentPose.theta);
        } else if(fabs(absoluteTheta - 180) < 10 || fabs(currentPose.theta + 180) < 10) {
            chassis.setPose(currentPose.x, -65 + disToWall, currentPose.theta);
        } else if(fabs(absoluteTheta + 90) < 10) {
            chassis.setPose(-65 + disToWall, currentPose.y, currentPose.theta);
        } else {
            controller.rumble("--");
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
            leftMotors.move(Speed + 10);
            rightMotors.move(Speed + 10);
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
            leftMotors.move(Speed - 10);
            rightMotors.move(Speed - 10);
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
    pros::delay(800);
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
    while(intake.get_actual_velocity() > 10) {
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
    chassis.setPose(-62, 0, 90); 
    intakeSpeed = 127;
	pros::delay(300);
    intakeSpeed = 0;
    driveToAxisPos('x', -48, true, 90, 127);
	chassis.turnToHeading(0, 600, {.minSpeed = 10, .earlyExitRange = 1});
    driveToAxisPos('y', -22, false, 0, -70);
	mogoMech.set_value(true);
    pros::delay(300);
    intakeSpeed = 127;
    chassis.turnToHeading(90, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.moveToPoint(-24, -26, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntil(6);
    intakeSpeed = 127;
    chassis.moveToPoint(0, -40, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(24, -48, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(110, 1500);
    doubleLBLoading = true;
    driveToAxisPos('x', 3, false, 110, -80);
    chassis.turnToHeading(180, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('y', -60, true, 180, 100);
    // resetting position before scoring first wall stake
    resetPos();
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    currentState = 2;
    doubleLBLoading = false;
    pros::delay(600);

    //SCORED THE FIRST WALLSTAKE!!

    driveToAxisPos('y', -48, false, 180, -80);
    currentState = 6;
    chassis.turnToHeading(-90, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToAxisPos('x', -60, true, -90, 60);
    resetPos();
    chassis.turnToPoint(-48, -60, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('x', -50, true, 135, 70);
    chassis.turnToHeading(70, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(400);
    currentState = 0;
    mogoMech.set_value(false);
    intakeSpeed = -127;
    pros::delay(150);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    chassis.follow(skillsPath4_txt, 18, 2000);
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    driveToDistance(20, 60, true);
    pros::delay(100);
    chassis.turnToHeading(180, 1500, {.minSpeed = 40, .earlyExitRange = 10});
    driveToAxisPos('y', 25, false, 180, -60);
    intakeSpeed = 0;
    mogoMech.set_value(true);
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x, currentPose.y, currentPose.theta);
    pros::delay(150);
    chassis.moveToPoint(-24, 24, 1500, {.minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntil(6);
    intakeSpeed = 127;
    chassis.moveToPoint(-10, 36, 1500, {.minSpeed = 60, .earlyExitRange = 5});
    chassis.moveToPoint(24, 50, 1500);
    chassis.waitUntilDone();
    chassis.turnToHeading(70, 500);
    doubleLBLoading = true;
    driveToAxisPos('x', 4, false, 70, -60);
    chassis.turnToHeading(0, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('y', 60, true, 0, 100);
    // resetting position before scoring first wall stake
    resetPos();
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    currentState = 2;
    doubleLBLoading = false;
    pros::delay(600);

    //SCORED THE SECOND WALLSTAKE!!    
    
    driveToAxisPos('y', 51, false, 0, -80);
    currentState = 6;
    chassis.turnToHeading(-90, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToAxisPos('x', -60, true, -90, 60);
    resetPos();
    chassis.turnToPoint(-48, 60, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('x', -50, true, 45, 70);
    chassis.turnToHeading(110, 600, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(500);
    currentState = 0;
    mogoMech.set_value(false);
    intakeSpeed = -127;
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
    intakeSpeed = 127;
    chassis.moveToPoint(24, 20, 1500);
    chassis.waitUntilDone();
    doubleLBLoading = true;
    chassis.turnToHeading(-90, 800, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('x', 60, false, -90, -70);
    mogoMech.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(5, 1500, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intakeSpeed = 0;
    //doinker.set_value(true);
    driveToAxisPos('y', 47, true, -0, 80);
    resetPos();
    doubleLBLoading = false;
    chassis.turnToHeading(-140, 500, {.minSpeed = 15});
    std::cout<<"turnedtoHeading(-140)" <<"\n";
    chassis.waitUntilDone();
    lemlib::Pose pose = chassis.getPose();
    std::cout<<"x:"<<pose.x <<"y:"<<pose.y<<"theta:"<<pose.theta<<"\n";
    // std::cout<<"waitUntilDone" <<"\n";
    mogoMech.set_value(false);
    std::cout<<"mogoFalse" <<"\n";
    leftMotors.move(-60);
    std::cout<<"leftmove(-60)" <<"\n";
    rightMotors.move(-60);
    std::cout<<"rightmove(-60)" <<"\n";
    pros::delay(500);
    std::cout<<"delay" <<"\n";
    std::cout<<"äfterdelay" <<"\n";
    pose = chassis.getPose();
    chassis.setPose(pose.x, pose.y, pose.theta);
    std::cout<<"driveD" <<"\n";
    std::cout<<"x:"<<pose.x <<"y:"<<pose.y<<"theta:"<<pose.theta<<"\n";    
    chassis.moveToPoint(51, 22, 1500, {.minSpeed = 10, .earlyExitRange = 1});
    std::cout<<"movetoPoint" <<"\n";
    chassis.waitUntilDone();
    std::cout<<"waitUntilDone" <<"\n";
    chassis.turnToHeading(0, 1500);
    chassis.waitUntilDone();
    driveToAxisPos('y', -1, false, 0, -60);
    mogoMech.set_value(true);
    chassis.turnToHeading(90, 1500, {.minSpeed = 10});
    chassis.waitUntilDone();
    leftMotors.move(90);
    rightMotors.move(90);
    pros::delay(800);
    leftMotors.move(0);
    rightMotors.move(0);
    driveToDistance(10, -60, true);
    currentState = 4;
    pros::delay(800);
    currentState = 3;
    driveToAxisPos('x', 47, false, -0, -80);
    currentState = 0;
    chassis.turnToHeading(0, 800, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToAxisPos('y', 57, true, -0, 80);
    
    chassis.moveToPoint(48, 0, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToPoint(24, -28, 1500, {.minSpeed = 10});
    chassis.waitUntilDone();
    driveToAxisPos('x', 24, true, 135, 70);
    chassis.turnToHeading(170, 700, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('y', -48, true, 170, 70);
    chassis.turnToHeading(86, 700, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('x', 60, true, 90, 127);
    chassis.waitUntilDone();
    chassis.waitUntilDone();
    driveToDistance(16, 80, true);
    driveToDistance(16, 80, true);
    resetPos();
    resetPos();
    driveToAxisPos('x', 49, false, 90, -50);
    chassis.turnToHeading(180, 700, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    driveToAxisPos('y', -59, true, 180, 70);
    chassis.turnToHeading(-65, 700, {.minSpeed = 10, .earlyExitRange = 1});
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(1000);
    leftMotors.move(0);
    rightMotors.move(0);
    chassis.moveToPoint(30, -30, 1000, {.minSpeed = 10});
    chassis.waitUntilDone();
    currentState = 6;
    chassis.turnToHeading(135, 800);
    chassis.waitUntilDone();
    intakeSpeed = -60;
    leftMotors.move(-45);
    rightMotors.move(-45);
    int hangCounter = 0;
    while(hangCounter < 20) {
        if(fabs(intake.get_actual_velocity()) < 5) {
            hangCounter++;
        } 
        pros::delay(20);
    }
    leftMotors.move(5);
    rightMotors.move(5);
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

void sawp() {
    chassis.setPose(64, -14, 29);
    currentState = 7;
    pros::delay(600);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(200);
    chassis.moveToPoint(32, -24, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(26, -24, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x + 0, currentPose.y - 0, currentPose.theta);

    currentState = 0;
    mogoMech.set_value(true);
    pros::delay(100);
    chassis.turnToHeading(180, 600);
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToAxisPos('y', -48, true, 180, 70);
    chassis.turnToHeading(45, 600, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.moveToPoint(48, 0, 1000);
    chassis.waitUntilDone();
    mogoMech.set_value(false);
    intakeSpeed = -50;
    pros::delay(250);
    chassis.turnToPoint(24, 24, 600, {.forwards = false});
    chassis.moveToPoint(27, 20, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = 50, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    chassis.turnToHeading(0, 600);
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToAxisPos('y', 47, true, 180, 70);
    chassis.turnToHeading(60, 600);
    chassis.moveToPoint(44, 60, 1000, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    leftMotors.move(80);
    rightMotors.move(80);
    pros::delay(1200);
    chassis.moveToPoint(64, 48, 1000, {.forwards = false, .minSpeed = 20, .earlyExitRange = 1});
    chassis.moveToPoint(65, 35, 1000, {.forwards = false, .minSpeed = 60});
    chassis.waitUntilDone();
    intakeSpeed = 0;
    chassis.turnToPoint(24,3, 1500, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntilDone();
    currentState = 2;
    chassis.moveToPoint(22, 22, 1000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    currentState = 4;
    mogoState = true;
}

void mainPosBlue() {
    colourSortingSelector = 1;
    chassis.setPose(64, -14, 29);
    currentState = 7;
    pros::delay(600);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(150);
    chassis.moveToPoint(38, -8, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    currentState = 0;
    chassis.turnToHeading(30, 350, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(20);
    rightMotors.move(20);
    doinker.set_value(true);
    pros::delay(100);

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x + 0, currentPose.y - 0, currentPose.theta);
    chassis.moveToPoint(28, -22, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(24, -24, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(150);
    chassis.turnToHeading(150, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(-45, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPose(8, -13, -70, 1500, {.maxSpeed = 60, .minSpeed = 40});
    chassis.waitUntilDone();
    doinker.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(33, -13, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToHeading(-180, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(100);
    intakeSpeed = 127;

    // OFFSETTING ODOMETRY
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x - 0, currentPose.y, currentPose.theta);

    chassis.moveToPoint(29, -28, 1500, {.maxSpeed = 50, .minSpeed = 30});
    chassis.moveToPoint(26, -45, 1500, {.maxSpeed = 50, .minSpeed = 30});
    
    chassis.moveToPoint(60, -50, 1500, {.minSpeed = 35, .earlyExitRange = 5});
    chassis.turnToHeading(90, 1500, {.minSpeed = 35, .earlyExitRange = 5});
    chassis.waitUntilDone();
    driveToDistance(6, 100, true);
    chassis.turnToHeading(160, 450, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(200);
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(700);
    leftMotors.move(-80);
    rightMotors.move(-80);
    pros::delay(400);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(150);
    leftMotors.move(40);
    rightMotors.move(60);
    pros::delay(700);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
    mogoState = true;
    colourSortingSelector = 0;
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
    chassis.moveToPoint(19, -19, 3000, {.minSpeed = 50, .earlyExitRange = 5});
    chassis.waitUntil(30);
    currentState = 2;
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
    chassis.setPose(-64, -14, -29);
    currentState = 7;
    pros::delay(600);
    leftMotors.move(-60);
    rightMotors.move(-60);
    pros::delay(250);
    chassis.moveToPoint(-41, -11, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    currentState = 0;
    chassis.turnToHeading(-57, 350, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(20);
    rightMotors.move(20);
    doinker.set_value(true);
    pros::delay(100);

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x + 0, currentPose.y - 0, currentPose.theta);
    chassis.moveToPoint(-28, -22, 1500, {.forwards = false, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-24, -24, 1500, {.forwards = false, .maxSpeed = 60, .minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(150);
    chassis.turnToHeading(-200, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(200);
    chassis.turnToHeading(45, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(true);
    chassis.moveToPose(-5, -1, 47, 1500, {.lead = .3, .maxSpeed = 60, .minSpeed = 20});
    chassis.moveToPoint(-31, -10, 1500, {.forwards = false, .minSpeed = 40, .earlyExitRange = 5});
    chassis.turnToHeading(150, 1500, {.minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doinker.set_value(false);
    pros::delay(100);
    intakeSpeed = 127;

    // OFFSETTING ODOMETRY
    currentPose = chassis.getPose();
    chassis.setPose(currentPose.x - 0, currentPose.y - 0, currentPose.theta);

    chassis.moveToPoint(-25, -35, 1500, {.maxSpeed = 40, .minSpeed = 20});
    chassis.moveToPoint(-26, -37, 1500, {.maxSpeed = 50, .minSpeed = 30});
    // chassis.turnToPoint(-48, -52, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    // chassis.moveToPoint(-48, -50, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    // chassis.waitUntil(5);
    // intakeSpeed = -127;
    chassis.moveToPoint(-55, -58, 1500, {.minSpeed = 30, .earlyExitRange = 5});
    // chassis.turnToHeading(-180, 500, {.minSpeed = 30});
    // driveToDistance(8, 100, true);
    chassis.turnToHeading(-110, 500, {.minSpeed = 30});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    leftMotors.move(50);
    rightMotors.move(50);
    pros::delay(200);
    leftMotors.move(100);
    rightMotors.move(100);
    pros::delay(900);
    leftMotors.move(-50);
    rightMotors.move(-90);
    pros::delay(400);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(150);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(700);
    leftMotors.move(-50);
    rightMotors.move(-50);
    pros::delay(300);
    leftMotors.move(0);
    rightMotors.move(0);
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
    mainPosRed();
    chassis.moveToPoint(-22, -22, 3000, {.minSpeed = 45, .earlyExitRange = 5});
    chassis.waitUntil(25);
    intakeSpeed = 0;
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 500);
    currentState = 7;
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
    chassis.moveToPose(-6, 51, 125, 1500, {.lead = 0.3, .minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(250);
    intakeSpeed = 127;
    chassis.moveToPoint(-32, 61, 1500, {.forwards = false, .minSpeed = 20, .earlyExitRange = 5});
    chassis.waitUntil(11);
    intakeSpeed = 20;
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
    chassis.moveToPoint(-58, 55, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(-90, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    intakeSpeed = 127;
    driveToDistance(6, 100, true);
    chassis.turnToHeading(-20, 500);
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
    leftMotors.move(50);
    rightMotors.move(100);
    pros::delay(700);
    leftMotors.move(-100);
    rightMotors.move(-100);
    pros::delay(100);
    chassis.turnToHeading(0, 300, {.minSpeed = 30});
    chassis.waitUntilDone();
    resetPos();
    chassis.moveToPoint(-58, 11, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    currentState = 6;
    chassis.turnToPoint(-72, 0, 500);
    chassis.waitUntilDone();
    doubleLBLoading = false;
    intakeSpeed = 127;
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    currentState = 7;
    driveD(-8);
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
    chassis.turnToPoint(-24, 0, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    leftMotors.move(60);
    rightMotors.move(60);
}

void mainNegBlue() {
    chassis.setPose(53.5, 32, -90);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    doinker.set_value(true);
    intakeSpeed = 127;    
    chassis.moveToPose(6, 46, -60, 1500, {.lead = 0.3, .minSpeed = 60});
    chassis.waitUntilDone();
    pros::delay(250);
    intakeSpeed = 127;

    // OFFSETTING ODOMETRY
    lemlib::Pose currentPose = chassis.getPose();
    chassis.setPose(currentPose.x - 1, currentPose.y + 1, currentPose.theta);

    chassis.moveToPoint(24, 24, 1500, {.forwards = false, .maxSpeed = 60});
    chassis.waitUntil(11);
    intakeSpeed = 0;
    chassis.waitUntilDone();
    mogoMech.set_value(true);
    pros::delay(200);
    intakeSpeed = 127;
    chassis.turnToHeading(-20,300);
    chassis.waitUntilDone();
    doinker.set_value(false);
    chassis.moveToPoint(24, 48, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    chassis.moveToPoint(45, 42, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(54, 56, 1500, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 5});
    chassis.turnToHeading(0, 1500, {.minSpeed = 30});
    chassis.waitUntilDone();
    driveToDistance(6, 100, true);
    chassis.turnToHeading(70, 500);
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
    pros::delay(450);
    leftMotors.move(0);
    rightMotors.move(0);
    pros::delay(1000);
    leftMotors.move(60);
    rightMotors.move(60);
    pros::delay(700);
    chassis.turnToHeading(0, 500, {.minSpeed = 20, .earlyExitRange = 1});
    resetPos();
    chassis.moveToPoint(54, 11, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    doubleLBLoading = false;
    intakeSpeed = 0;
    currentState = 6;
    chassis.turnToPoint(72, 1, 500);
    chassis.waitUntilDone();
    leftMotors.move(127);
    rightMotors.move(127);
    pros::delay(500);
    leftMotors.move(0);
    rightMotors.move(0);
    driveD(-7.5);
    currentState = 7;
    pros::delay(600);
    mogoState = true;
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
    leftMotors.move(-70);
    rightMotors.move(-70);
    pros::delay(200);
    currentState = 0;
    chassis.turnToPoint(24, 0, 1500, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.waitUntilDone();
    leftMotors.move(60);
    rightMotors.move(60);
}