#ifndef auton_functions_cpp
#define auton_functions_cpp

#include "globals.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/misc/lv_math.h"
#include <cmath>

inline lemlib::PID angularPID(1.6, 0.0, 12.0, 0, false);
inline lemlib::PID linearPID(3.5, 0.0, 12.0, 0, false);

// turns to a desired heading
inline void turnToHeading(float targetHeading) {
    float TARGET_THRESHOLD = 2; // degrees
    int TIME_BEFORE_EXIT = 200; // in milliseconds
    int TIMEOUT = 2000; // milliseconds

    // Normalize target heading to [0, 360)
    targetHeading = fmod(targetHeading + 360.0, 360.0);

    float currentHeading = inertial_sensor.get_heading();
    float error = targetHeading - currentHeading;

    // Shortest path
    if (error > 180.0) error -= 360.0;
    else if (error < -180.0) error += 360.0;

    bool inSettleZone = false;
    uint32_t settleStartTime;
    uint32_t startTime = pros::millis();

    // while the robot is not at target heading and while the timer has time left on the clock
    while (true) {
        currentHeading = inertial_sensor.get_heading();
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
        leftMotors.move(targetSpeed);
        rightMotors.move(-targetSpeed);
        
        pros::delay(20);
    }
    
    // resetting things
    angularPID.reset();
    leftMotors.move(0);
    rightMotors.move(0);
}

// turns to a desired point. may be ran in conjunction with moveToPoint
inline void turnToPoint(float targetX, float targetY, bool direction) {
    lemlib::Pose currentPose = chassis.getPose();

    // find the heading from the atan function (returns trig-angle type style)
    float targetHeading = lv_atan2(targetX - currentPose.x, targetY - currentPose.y);

    //converting to navigation angle style
    targetHeading = fmod((450 - targetHeading), 360);

    // adjust heading for reverse situation
    if(direction == false) { // false = reverse
        targetHeading = fmod(targetHeading + 180.0, 360.0);
    }

    // use turnToHeading to turn the robot
    turnToHeading(targetHeading);
}

// moves to a desired point. may run turnToPoint before the robot starts moving
inline void moveToPoint(float targetX, float targetY, bool direction, bool turnBeforeMoving) {
    if(turnBeforeMoving == true) {
        turnToPoint(targetX, targetY, direction);
    }

    float TARGET_THRESHOLD = 2; // inches
    int TIME_BEFORE_EXIT = 350; // in milliseconds
    int TIMEOUT = 2500; // milliseconds

    lemlib::Pose currentPose = chassis.getPose();
    // measuring point-to-point distance of destinationPose and currentPose
    float linearError = sqrt(pow(targetX - currentPose.x, 2) + pow(targetY - currentPose.y, 2));

    bool inSettleZone = false;
    uint32_t settleStartTime;
    uint32_t startTime = pros::millis();

    while (true) {
        currentPose = chassis.getPose();
        linearError = sqrt(pow(targetX - currentPose.x, 2) + pow(targetY - currentPose.y, 2));

        float currentHeading = inertial_sensor.get_heading();
        // find the heading from the atan function (returns trig-angle type style)
        float targetHeading = lv_atan2(targetX - currentPose.x, targetY - currentPose.y);
        //converting to navigation angle style
        targetHeading = fmod((450 - targetHeading), 360);
        
        if(direction == false) { // false = reverse
            targetHeading = fmod(targetHeading + 180.0, 360.0);
        }

        float angularError = targetHeading - currentHeading;
        if (angularError > 180.0) angularError -= 360.0;
        else if (angularError < -180.0) angularError += 360.0;
        
        // check to see if in settle zone
        if (fabs(linearError) <= TARGET_THRESHOLD) {
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
        float targetSpeed = linearPID.update(linearError);
        float angleCorrect = angularPID.update(angularError);

        // makes the angle correct not trigger when close to the target
        if(linearError < 12) {
            float reductionFactor = (linearError / 12.0) * (linearError / 12.0); // quadratic curve
            angleCorrect *= reductionFactor;
        }
        if(linearError < 3) {
            angleCorrect = 0;
        }

        if(direction == false) { // false = reverse
            targetSpeed = -targetSpeed;
        }

        if(targetSpeed > 127) targetSpeed = 127;
        if(targetSpeed < -127) targetSpeed = -127;

        float leftMotorPower = targetSpeed + angleCorrect;
        float rightMotorPower = targetSpeed - angleCorrect;

        // Find the maximum absolute power
        float maxPower = fmax(fabs(leftMotorPower), fabs(rightMotorPower));

        // scale down if needed
        if(maxPower > 127) {
            float scale = 127.0 / maxPower;
            leftMotorPower *= scale;
            rightMotorPower *= scale;
        }

        leftMotors.move(leftMotorPower);
        rightMotors.move(rightMotorPower);
        
        pros::delay(20);
    }
    
    // resetting things
    linearPID.reset();
    angularPID.reset();
    leftMotors.move(0);
    rightMotors.move(0);
}

inline void driveToDistance() {
    
}
#endif


