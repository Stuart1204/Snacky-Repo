#ifndef auton_functions_cpp
#define auton_functions_cpp

#include "globals.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/misc/lv_math.h"
#include <cmath>

inline lemlib::PID angularPID(1.6, 0.0, 12.0, 0, false);
inline lemlib::PID linearPID(1.6, 0.0, 12.0, 0, false);

// turns to a desired heading
inline void turnToHeading(float targetHeading) {
    float TARGET_THRESHOLD = 2;
    float TIME_BEFORE_EXIT = 200; // in milliseconds

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
        
        // timeout (3 seconds)
        if (pros::millis() - startTime > 3000) {
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
inline void turnToPoint(float targetX, float targetY) {
    lemlib::Pose currentPose = chassis.getPose();

    // find the heading from the atan function (returns trig-angle type style)
    float targetHeading = lv_atan2(targetX - currentPose.x, targetY - currentPose.y);

    //converting to navigation angle style
    targetHeading = fmod((450 - targetHeading), 360);

    // use turnToHeading to turn the robot
    turnToHeading(targetHeading);
}

// moves to a desired point. may run turnToPoint before the robot starts moving
inline void moveToPoint(float targetX, float targetY) {
    
}

#endif


