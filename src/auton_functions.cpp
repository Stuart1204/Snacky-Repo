#ifndef auton_functions_cpp
#define auton_functions_cpp

#include "globals.hpp"
#include "other_functions.cpp"
#include "intake.cpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cmath>
#include <iostream>

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
    float targetHeading = lv_atan2(targetY - currentPose.y, targetX - currentPose.x);
    //printf("lv_atan2 result: %f\n", targetHeading);

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

        if (pros::millis() - startTime > 2000) {
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

inline void driveDistance(float targetDist, int maxSpeed) {
    float TARGET_THRESHOLD = 3; // inches
    int TIME_BEFORE_EXIT = 100; // in milliseconds
    int TIMEOUT = 2500; // milliseconds

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

inline void goalShake(int TIMEOUT) {
    int MOTOR_SPEED = 60;
    uint32_t startTime = pros::millis();
    bool direction = false;
    bool driveBack = false;

    while(true) {
        if(!driveBack) {
            if(direction) {
                leftMotors.move(10);
                rightMotors.move(-120);
                pros::delay(200);
                direction = !direction;
            } else {
                leftMotors.move(-120);
                rightMotors.move(10);
                pros::delay(200);
                direction = !direction;
            }
        } else {
            leftMotors.move(-90);
            rightMotors.move(-90);
        }

        if (pros::millis() - startTime > TIMEOUT - 500) {
            driveBack = true;
        }

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }
    }
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void waitAtMatchload() {
    waitAtMatchload(450);
}

inline void skillsWaitAtMatchload() {
    waitAtMatchload(1800);
}

inline void driveInParkZone() {
    int MOTOR_SPEED = 60;
    int TIMEOUT = 2500;
    uint32_t startTime = pros::millis();
    int startValue = vertical_rotation.get_position() / 100;

    while(true) {
        if(vertical_rotation.get_position() / 100 > startValue + 900) {
            break;
        }
        leftMotors.move(MOTOR_SPEED);
        rightMotors.move(MOTOR_SPEED);

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }
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

inline void driveToGoalDistance() {
    driveToDistance(22);
    resetPos();
}

inline void skillsClearParkZone() {
    int MOTOR_SPEED = 60;
    int TIMEOUT = 2500;
    uint32_t startTime = pros::millis();
    int startValue = vertical_rotation.get_position() / 100;

    while(true) {
        if(vertical_rotation.get_position() / 100 > startValue + 1650) {
            break;
        }
        leftMotors.move(MOTOR_SPEED);
        rightMotors.move(MOTOR_SPEED);

        if (pros::millis() - startTime > TIMEOUT) {
            break;
        }
    }
    leftMotors.move(0);
    rightMotors.move(0);

    moveDrive(50, 400);
}

inline bool getGyroValue = false;
inline int gyroValue;
inline bool onFirstStep = true;
inline bool onSecondStep = false;

inline void startAutoWing() {
    getGyroValue = true;
    enableAutoWing = true;
    onFirstStep = true;
    onSecondStep = false;
}

inline void autoWing() {
    if(getGyroValue) {
        gyroValue = inertial_sensor.get_rotation();
        getGyroValue = false;
    }
    if(enableAutoWing) {
        if(onFirstStep) {
            if(inertial_sensor.get_rotation() > gyroValue - 20) {
                leftMotors.move(-120);
                rightMotors.move(120);
            } else {
                onFirstStep = false;
                onSecondStep = true;
            }
        }
        if(onSecondStep) {
            if(inertial_sensor.get_rotation() < gyroValue - 20) {
                leftMotors.move(90);
                rightMotors.move(10);
            } else {
                enableAutoWing = false;
                leftMotors.move(-15);
                rightMotors.move(15);
            }
        }
    }
}

inline void autonAutoWing() {
    if(getGyroValue) {
        gyroValue = inertial_sensor.get_rotation();
        getGyroValue = false;
    }
    if(enableAutoWing) {
        if(onFirstStep) {
            if(inertial_sensor.get_rotation() > gyroValue - 38) {
                leftMotors.move(-80);
                rightMotors.move(80);
            } else {
                onFirstStep = false;
                onSecondStep = true;
            }
        }
        if(onSecondStep) {
            if(inertial_sensor.get_rotation() < gyroValue - 38) {
                leftMotors.move(90);
                rightMotors.move(10);
            } else {
                enableAutoWing = false;
                leftMotors.move(-15);
                rightMotors.move(15);
            }
        }
    }
}

inline void autonAutoWingOpposite() {
    if(getGyroValue) {
        gyroValue = inertial_sensor.get_rotation();
        getGyroValue = false;
    }
    if(enableAutoWing) {
        if(onFirstStep) {
            if(inertial_sensor.get_rotation() < gyroValue + 38) {
                leftMotors.move(80);
                rightMotors.move(-80);
            } else {
                onFirstStep = false;
                onSecondStep = true;
            }
        }
        if(onSecondStep) {
            if(inertial_sensor.get_rotation() > gyroValue + 38) {
                leftMotors.move(10);
                rightMotors.move(90);
            } else {
                enableAutoWing = false;
                leftMotors.move(15);
                rightMotors.move(-15);
            }
        }
    }
}

#endif


