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
#include "globals.hpp"
#include "intake.cpp"
#include "piston.cpp"
#include "auton_functions.cpp"
#include "other_functions.cpp"

inline void testingAuto() {
    chassis.setPose(0,0,0);
    moveToPoint(24, 24, true, false);
}

inline void leftSideAuto() {
    chassis.setPose(0,0,0);
}

inline void rightSideAuto() {

}

inline void SAWP() {
    chassis.setPose(0,0,0);
    moveToPoint(-35, 10, true, false);
    turnToHeading(55);
    setWings(true);
    intake.setState(IntakeState::INTAKE);
    driveDistance(15, 50); // picking up the three clump of balls
    moveToPoint(-40, 40, true, true);
    turnToHeading(0);
    driveToDistance(22);
    turnToHeading(-90);
    driveToGoal(false); // driving to the long goal
    intake.setState(IntakeState::SCOREHIGH); // scoring balls onto long goal
    pros::delay(3000);
    tongue.setState(true);
    intake.setState(IntakeState::INTAKE);
    setWings(true);
    moveToPoint(-56, 48, true, true);
    leftMotors.move(30);
    rightMotors.move(30); // collecting matchloads
    waitAtMatchload(allianceColour);
}

