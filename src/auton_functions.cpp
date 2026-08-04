#ifndef auton_functions_cpp
#define auton_functions_cpp


#include "globals.hpp"
#include "other_functions.cpp"
#include "intake.cpp"
#include "elevator.cpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "piston.cpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cmath>
#include <iostream>


// Define a simple struct for goal coordinates
struct Point {int x; int y;};

// Array of all 9 goals on the Override field
const Point GOALS[9] = {
    {0, 0},           // Center Goal
    {-48, 24},        // Neutral Red 1
    {-48, -24},       // Red 1
    {-24, -48},       // Red 2
    {24, -48},        // Neutral Red 2
    {48, -24},        // Neutral Blue 1
    {48, 24},         // Blue 1
    {24, 48},         // Blue 2
    {-24, 48}         // Neutral Blue 2
};

inline void goalReset() {
    // --- PHYSICAL PARAMETERS ---
    const float GOAL_CENTER_TO_FACE = 2.805; // inches
    const float ROBOT_BACK_TO_CENTER = 5.0;   // inches

    const float TOTAL_OFFSET = GOAL_CENTER_TO_FACE + ROBOT_BACK_TO_CENTER;

    lemlib::Pose currentPose = chassis.getPose();

    Point closestGoal  = GOALS[0];
    float minDistance  = 9999.0f;

    for (int i = 0; i < 9; i++) {
        float dx   = currentPose.x - GOALS[i].x;
        float dy   = currentPose.y - GOALS[i].y;
        float dist = sqrtf(dx * dx + dy * dy);

        if (dist < minDistance) {
            minDistance = dist;
            closestGoal = GOALS[i];
        }
    }

    // -------------------------------------------------------
    // 3. Snap heading to nearest 45° and compute new pose
    // -------------------------------------------------------
    // LemLib: 0° = North (+Y), 90° = East (+X), clockwise positive

    float thetaRad = currentPose.theta*(M_PI/180.0f);

    float newX = closestGoal.x + TOTAL_OFFSET * sin(thetaRad);
    float newY = closestGoal.y + TOTAL_OFFSET * cos(thetaRad);

    
    // -------------------------------------------------------
    // 4. Reset odometry (keep theta from IMU, update X & Y)
    // -------------------------------------------------------
    printf("========== GOAL RESET ==========\n");
    printf("GoalX:        %d\n",   closestGoal.x);
    printf("GoalY:        %d\n",   closestGoal.y);
    printf("currX:        %.3f\n", currentPose.x);
    printf("currY:        %.3f\n", currentPose.y);
    printf("currTheta:    %.3f\n", currentPose.theta);
    printf("newX:         %.3f\n", newX);
    printf("newY:         %.3f\n", newY);
    printf("================================\n");

    chassis.setPose(newX, newY, currentPose.theta);
    controller.rumble(".");
}


inline void driveDistance(float targetDist, int maxSpeed) {
    float TARGET_THRESHOLD = 5; // inches
    int TIME_BEFORE_EXIT = 0; // in milliseconds
    int TIMEOUT = 2500; // milliseconds
    lemlib::PID linearPID{5.6,0,29, 0};
    bool inSettleZone = false;
    uint32_t settleStartTime;
    uint32_t startTime = pros::millis();

    int offsetDegrees = vertical_rotation.get_position();

    while(true) {
        float currentDist = (vertical_rotation.get_position() - offsetDegrees) * CENTIDEG_TO_INCHES;
        float error = targetDist - currentDist;

        // check to see if in settle zone
        if (fabs(error) <= TARGET_THRESHOLD) {
            if (!inSettleZone) {
                settleStartTime = pros::millis();
                inSettleZone = true;
            }
            
            // wait in the settle zone
            if (pros::millis() - settleStartTime >= TIME_BEFORE_EXIT) {
                break; // exit loop
            }
        } else {
            inSettleZone = false; // reset the timer if we leave the settle zone
        }

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }

        float targetSpeed = linearPID.update(error);

        if(targetSpeed > maxSpeed) targetSpeed = maxSpeed;
        if(targetSpeed < -maxSpeed) targetSpeed = -maxSpeed;
        if(targetSpeed < 20 && targetSpeed > 0) targetSpeed = 20;
        if(targetSpeed > -20 && targetSpeed < 0) targetSpeed = -20;
        

        leftMotors.move(targetSpeed);
        rightMotors.move(targetSpeed);

        pros::delay(20);
    }
    linearPID.reset();
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void driveDistance(float targetDist) {
    driveDistance(targetDist, 127);
}



inline lemlib::PID angularPID(4.0, 0.0, 17.0, 0, false);
inline lemlib::PID linearPID(7.5, 0.0, 26.0, 0, false);

inline void turnToHeading(float targetHeading, bool exitEarly) {
    float TARGET_THRESHOLD; // degrees
    int MIN_SPEED = 10;
    int TIME_BEFORE_EXIT = 100; // in milliseconds
    int TIMEOUT = 1000; // milliseconds

    if(exitEarly) {
        TARGET_THRESHOLD = 7.5;
    } else {
        TARGET_THRESHOLD = 2.5;
    }

    // Normalize target heading to [0, 360)
    targetHeading = fmod(fmod(targetHeading, 360.0) + 360.0, 360.0);

    float currentHeading = chassis.getPose().theta;
    currentHeading = fmod(fmod(currentHeading, 360.0) + 360.0, 360.0);

    float error = targetHeading - currentHeading;

    // Shortest path
    if (error > 180.0) error -= 360.0;
    else if (error < -180.0) error += 360.0;

    if(fabs(error) < 3) {
        TARGET_THRESHOLD = 2;
    } // more precise if robot is already close to error

    bool inSettleZone = false;
    uint32_t settleStartTime;
    uint32_t startTime = pros::millis();

    // while the robot is not at target heading and while the timer has time left on the clock
    while (true) {
        currentHeading = chassis.getPose().theta;
        currentHeading = fmod(fmod(currentHeading, 360.0) + 360.0, 360.0);
        error = targetHeading - currentHeading;
        
        // calculate shortest path
        if (error > 180.0) error -= 360.0;
        else if (error < -180.0) error += 360.0;
        
        // check to see if in settle zone
        if (fabs(error) <= TARGET_THRESHOLD) {
            if (!inSettleZone) {
                settleStartTime = pros::millis();
                inSettleZone = true;
            }
            
            // wait in the settle zone
            if (pros::millis() - settleStartTime >= TIME_BEFORE_EXIT) {
                break; // exit loop
            }
        } else {
            inSettleZone = false; // reset the timer if we leave the settle zone
        }
        
        // timeout (2 seconds)
        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }
        
        // calculating motor power using PID
        float targetSpeed = angularPID.update(error);
        
        // moving motors
        if(fabs(targetSpeed) < MIN_SPEED) {
            targetSpeed *= 1.6;
        }
        leftMotors.move(targetSpeed);
        rightMotors.move(-targetSpeed);
        
        pros::delay(20);
    }
    
    // resetting things
    angularPID.reset();
    leftMotors.move(0);
    rightMotors.move(0);
}
// turns to a desired heading
inline void turnToHeading(float targetHeading) {
    turnToHeading(targetHeading, false);
}

// turns to a desired point. may be ran in conjunction with moveToPoint
inline void turnToPoint(float targetX, float targetY, bool direction) {
    lemlib::Pose currentPose = chassis.getPose();

    // find the heading from the atan function (returns trig-angle type style)
    float targetHeading = std::atan2(targetY - currentPose.y, targetX - currentPose.x) * (180.0 / M_PI);
    //printf("atan2 result: %f\n", targetHeading);

    //converting to navigation angle style
    targetHeading = fmod((450 - targetHeading), 360);
    //printf("After conversion: %f\n", targetHeading);

    // adjust heading for reverse situation
    if(direction == false) { // false = reverse
        targetHeading = fmod(targetHeading + 180.0, 360.0);
    }

    //printf("Final target heading: %f\n", targetHeading);
    // use turnToHeading to turn the robot
    turnToHeading(targetHeading, true);
}

inline void moveToPoint(float targetX, float targetY, bool direction, bool turnBeforeMoving, bool blendMovements, float maxSpeed) {
    if(turnBeforeMoving == true) {
        turnToPoint(targetX, targetY, direction);
    }
    if(blendMovements) {
        chassis.moveToPoint(targetX, targetY, 2500, {.forwards = direction, .maxSpeed = maxSpeed, .minSpeed = 30, .earlyExitRange = 5});
    } else {
        chassis.moveToPoint(targetX, targetY, 2500, {.forwards = direction, .maxSpeed = maxSpeed,});
    }
    chassis.waitUntilDone();
}

inline void moveToPoint(float targetX, float targetY, bool direction, bool turnBeforeMoving, bool blendMovements) {
    moveToPoint(targetX, targetY, direction, turnBeforeMoving, blendMovements, 127);
}

inline void moveToPoint(float targetX, float targetY, bool direction, bool turnBeforeMoving) {
    moveToPoint(targetX, targetY, direction, turnBeforeMoving, false);
}

inline void resetPos() {
    const float FIELD_WALL = 70.0; // inches
    const float ANGLE_TOLERANCE = 30.0; // degrees
    
    lemlib::Pose currentPose = chassis.getPose();
    
    // Normalize angle to -180 to 180 range
    float normalizedAngle = fmod(currentPose.theta + 180.0, 360.0) - 180.0;

    if(!frontDistSensor.is_installed()) {
        return;
    }

    float currentDistance;
    if(frontDistSensor.get() != 9999) {
        currentDistance = (frontDistSensor.get_distance() * MM_TO_INCHES) + FRONT_DIST_OFFSET; 

        // Determine which wall we're facing based on angle
        if (fabs(normalizedAngle) < ANGLE_TOLERANCE) {
            // Facing forward (0°)
            float offsetAngle = fabs(normalizedAngle);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, FIELD_WALL - currentDistance, chassis.getPose().theta);
            // controller.rumble("-");
        } else if (fabs(normalizedAngle - 90) < ANGLE_TOLERANCE) {
            // Facing right (90°)
            float offsetAngle = fabs(normalizedAngle - 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(FIELD_WALL - currentDistance, currentPose.y, chassis.getPose().theta);
            // controller.rumble("-");
        } else if (fabs(normalizedAngle + 90) < ANGLE_TOLERANCE) {
            // Facing left (-90°)
            float offsetAngle = fabs(normalizedAngle + 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(-FIELD_WALL + currentDistance, currentPose.y, chassis.getPose().theta);
            //controller.rumble("-");
        } else if (fabs(fabs(normalizedAngle) - 180) < ANGLE_TOLERANCE) {
            // Facing backward (180° or -180°)
            float offsetAngle = fabs(fabs(normalizedAngle) - 180);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, -FIELD_WALL + currentDistance, chassis.getPose().theta);
            //controller.rumble("-");
        }
    }
}

inline void turnWithSpeeds(int leftDriveSpeed, int rightDriveSpeed, float targetAngle) {
    uint32_t startTime = pros::millis();
    
    while(true) {
        float normalizedAngle = fmod(chassis.getPose().theta + 180.0, 360.0) - 180.0;
        float error = targetAngle - normalizedAngle;

        if (pros::millis() - startTime > 1200) {
            break;
        }

        leftMotors.move(leftDriveSpeed);
        rightMotors.move(rightDriveSpeed);

        if(fabs(error) < 2) {
            break;
        }
    }
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void leftResetPos() {
    const float FIELD_WALL = 70.0; // inches
    const float ANGLE_TOLERANCE = 25.0; // degrees
    
    lemlib::Pose currentPose = chassis.getPose();
    
    // Normalize angle to -180 to 180 range
    float normalizedAngle = fmod(currentPose.theta + 180.0, 360.0) - 180.0;

    if(!leftDistSensor.is_installed()) {
        return;
    }

    float currentDistance;
    if(leftDistSensor.get() != 9999) {
       currentDistance = (leftDistSensor.get_distance() * MM_TO_INCHES) + LEFT_DIST_OFFSET; 

        // Determine which wall we're facing based on angle
        if (fabs(normalizedAngle - 90) < ANGLE_TOLERANCE) {
            // Facing right (90°)
            float offsetAngle = fabs(normalizedAngle - 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, FIELD_WALL - currentDistance, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(fabs(normalizedAngle) - 180) < ANGLE_TOLERANCE) {
            // Facing right (90°)
            float offsetAngle = fabs(fabs(normalizedAngle) - 180);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(FIELD_WALL - currentDistance, currentPose.y, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(normalizedAngle) < ANGLE_TOLERANCE) {
            // Facing left (-90°)
            float offsetAngle = fabs(normalizedAngle);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(-FIELD_WALL + currentDistance, currentPose.y, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(normalizedAngle + 90) < ANGLE_TOLERANCE) {
            // Facing backward (180° or -180°)
            float offsetAngle = fabs(normalizedAngle + 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, -FIELD_WALL + currentDistance, chassis.getPose().theta);
            controller.rumble("-");
        }
    }
}

inline void rightResetPos() {
    const float FIELD_WALL = 70.0; // inches
    const float ANGLE_TOLERANCE = 25.0; // degrees
    
    lemlib::Pose currentPose = chassis.getPose();
    
    // Normalize angle to -180 to 180 range
    float normalizedAngle = fmod(currentPose.theta + 180.0, 360.0) - 180.0;

    if(!rightDistSensor.is_installed()) {
        return;
    }

    float currentDistance;
    if(rightDistSensor.get() != 9999) {
       currentDistance = (rightDistSensor.get_distance() * MM_TO_INCHES) + RIGHT_DIST_OFFSET; 

        // Determine which wall we're facing based on angle
        if (fabs(normalizedAngle + 90) < ANGLE_TOLERANCE) {
            // Facing right (90°)
            float offsetAngle = fabs(normalizedAngle + 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, FIELD_WALL - currentDistance, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(normalizedAngle) < ANGLE_TOLERANCE) {
            // Facing right (90°)
            float offsetAngle = fabs(normalizedAngle);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(FIELD_WALL - currentDistance, currentPose.y, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(fabs(normalizedAngle) - 180) < ANGLE_TOLERANCE) {
            // Facing left (-90°)
            float offsetAngle = fabs(fabs(normalizedAngle) - 180);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(-FIELD_WALL + currentDistance, currentPose.y, chassis.getPose().theta);
            controller.rumble("-");
        } else if (fabs(normalizedAngle - 90) < ANGLE_TOLERANCE) {
            // Facing backward (180° or -180°)
            float offsetAngle = fabs(normalizedAngle - 90);
            currentDistance *= cos(offsetAngle * DEG_TO_RAD);
            chassis.setPose(currentPose.x, -FIELD_WALL + currentDistance, chassis.getPose().theta);
            controller.rumble("-");
        }
    }
}



inline void driveToGoal(bool direction, int speed) {
    const float SPEED_THRESHOLD = 100; // out if 600
    const int MOTOR_SPEED = speed; // out of 127
    const int TIMEOUT = 1000;

    bool reachedTargetSpeed = false;
    uint32_t startTime = pros::millis();

    while(true) {
        float currentSpeed = fmin(leftMotors.get_actual_velocity(), rightMotors.get_actual_velocity());

        if(pros::millis() - startTime > 300) {
            reachedTargetSpeed = true;
        }

        if(fabs(currentSpeed) < SPEED_THRESHOLD && reachedTargetSpeed == true) {
            break;
        }

        if(direction) {
            leftMotors.move(MOTOR_SPEED);
            rightMotors.move(MOTOR_SPEED);
        } else {
            leftMotors.move(-MOTOR_SPEED);
            rightMotors.move(-MOTOR_SPEED);
        }

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }

        pros::delay(20);
    }
    if(direction) {
        leftMotors.move(10);
        rightMotors.move(10);
    } else {
       leftMotors.move(-10);
        rightMotors.move(-10); 
    }
}

inline void driveToGoal(bool direction) {
    driveToGoal(direction, 60);
}

inline void waitAtMatchload(int timeout) {
    const int TIMEOUT = timeout;

    uint32_t startTime = pros::millis();
    int counter = 0;
    bool direction = true;

    while(true) {
        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }

        counter++;
        if(direction == true) {
            if(counter < 8) {
                leftMotors.move(55);
                rightMotors.move(55);
            } else {
                counter = 0;
                direction = false;
            }
        } else {
            if(counter < 2) {
                leftMotors.move(-15);
                rightMotors.move(-15);
            } else {
                counter = 0;
                direction = true;
            }
        }

        pros::delay(20);
    }
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void moveDrive(int speed, int timeout) {
    leftMotors.move(speed);
    rightMotors.move(speed);
    pros::delay(timeout);
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void driveToDistance(float distance) {
    float TARGET_THRESHOLD = 0.25; // inches
    int TIME_BEFORE_EXIT = 50; // in milliseconds
    int TIMEOUT = 1000; // milliseconds
    int maxSpeed = 60;
    int minSpeed = 5;

    bool inSettleZone = false;
    uint32_t settleStartTime;
    uint32_t startTime = pros::millis();

    int offsetDegrees = vertical_rotation.get_position();

    while(true) {
        float currentDistance = (frontDistSensor.get_distance() * MM_TO_INCHES) + FRONT_DIST_OFFSET; 
        float error = currentDistance - distance;

        // check to see if in settle zone
        if (fabs(error) <= TARGET_THRESHOLD) {
            if (!inSettleZone) {
                settleStartTime = pros::millis();
                inSettleZone = true;
            }
            
            // wait in the settle zone
            if (pros::millis() - settleStartTime >= TIME_BEFORE_EXIT) {
                break; // exit loop
            }
        } else {
            inSettleZone = false; // reset the timer if we leave the settle zone
        }

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }

        float targetSpeed = linearPID.update(error);

        if(targetSpeed > maxSpeed) targetSpeed = maxSpeed;
        if(targetSpeed < -maxSpeed) targetSpeed = -maxSpeed;
        
        if(targetSpeed < minSpeed && targetSpeed > 0) targetSpeed = minSpeed;
        if(targetSpeed > -minSpeed && targetSpeed < 0) targetSpeed = -minSpeed;

        leftMotors.move(targetSpeed);
        rightMotors.move(targetSpeed);

        pros::delay(20);
    }
    linearPID.reset();
    leftMotors.move(0);
    rightMotors.move(0);
}


inline void skillsAcquireNeutralMatchload() {
    moveToPoint(-48, 56, true, true);
    turnToHeading(-90, true);
    rightResetPos();
    isNeutralGoal = true;
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);

    moveToPoint(-62, 56, true, false);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    turnToHeading(-90, true);
    moveDrive(30, 750);
	moveDrive(-12, 150);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2750) {
            break;
        }
    }
}
inline void skillsScoreOnNeutralGoal() {
    rightResetPos();
    moveDrive(-50, 300);
    moveToPoint(-50, 33, false, true, true, 70);
    moveDrive(-35, 500);
    leftMotors.move(-30);
    rightMotors.move(-30);
    elevator.setState(ElevatorState::SCORE);

    goalReset();
    pros::delay(350);
}

inline void skillsAquireAllianceMatchload() {
    moveToPoint(-48, -60, true, true);
    turnToHeading(-90, true);
    leftResetPos();
    isNeutralGoal = false;
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);

    moveToPoint(-60, -60, true, false);
    intake.setState(IntakeState::INTAKE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    turnToHeading(-90, true);
    moveDrive(30, 500);
	moveDrive(-12, 150);
    uint32_t startTime = pros::millis();
    while(elevator.getState() != ElevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2750) {
            break;
        }
    }
}



inline void skillsScoreOnAllianceGoal() {
    leftResetPos();
    moveDrive(-50, 300);
    moveToPoint(-50, -33, false, true, true, 70);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(500);
    elevator.setState(ElevatorState::SCORE);
    goalReset();
    pros::delay(350);
}

#endif


