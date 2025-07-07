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
    moveToPoint(24, 24, true, true);
}

inline void leftSideAuto() {
    chassis.setPose(0,0,0);
}

inline void rightSideAuto() {

}

inline void SAWP() {
    chassis.setPose(-48, 24 - 6.75 - .5 ,90);
    intake.setState(IntakeState::IDLE);
    moveToPoint(-35, 18, true, false);
    turnToHeading(55);
    setWings(true);
    intake.setState(IntakeState::INTAKE);
    driveDistance(17, 30); // picking up the three clump of balls
    moveToPoint(-37, 48, false, true);
    turnToHeading(-90);
    intake.setState(IntakeState::IDLE);
    setWings(false);
    driveToGoal(false); // driving to the long goal
    intake.setState(IntakeState::SCOREHIGH); // scoring balls onto long goal
    pros::delay(1000);
    std::cout << "afterDelay";    
    tongue.setState(true);
    std::cout << "afterTongue";    
    intake.setState(IntakeState::INTAKE);
    std::cout << "afterIntake";    
    setWings(true);
    std::cout << "afterWings";    
    moveToPoint(-57, 48, true, true);
    leftMotors.move(80);
    rightMotors.move(80); // collecting matchloads
    pros::delay(300);
    waitAtMatchload(allianceColour);
    intake.setState(IntakeState::IDLE);
    driveDistance(-10);
    tongue.setState(false);
    moveToPoint(-14, 14, false, true);

}

