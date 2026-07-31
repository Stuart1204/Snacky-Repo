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

inline void skillsAuton() {

}

inline void SAWP() {

}

inline void TAWP() { // teamate has to score one pin and get toggle
    chassis.setPose(-60.5, 9, -128);
    

    isNeutralGoal = false;
    elevator.setState(ElevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(-25, 300, {.minSpeed = 80});
    chassis.moveToPoint(-62.5, -9, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    elevator.setState(ElevatorState::PIN_HEIGHT);
    elevator.goToPinHeight(0);
    matchloadMode = true;
    intakeLift.setState(true);
    hood.setState(true);
    chassis.swingToHeading(-90, DriveSide::RIGHT, 700, {.minSpeed = 90, .earlyExitRange = 10});
    chassis.waitUntilDone();
    moveDrive(0, 300);
    elevator.setState(ElevatorState::SCORE);
    lemlib::Pose pose = chassis.getPose();
    printf("currX:        %.3f\n", pose.x);
    printf("currY:        %.3f\n", pose.y);
    goalReset();
    pros::delay(350);
    elevator.setState(ElevatorState::RESET);
    turnToHeading(-90, true);
    leftMotors.move(30);
    rightMotors.move(30);
    intake.setState(IntakeState::INTAKE);
    pros::delay(2500);
    moveDrive(-60, 500);
    elevator.setState(ElevatorState::SCORE);
    pose = chassis.getPose();
    printf("currX:        %.3f\n", pose.x);
    printf("currY:        %.3f\n", pose.y);
    goalReset();
    pros::delay(350);
    elevator.setState(ElevatorState::RESET);
    moveDrive(50, 300);
    elevator.setState(ElevatorState::GROUND_PICKUP);
    moveToPoint(-48, -42, false, true, true);
    leftMotors.move(-10);
	rightMotors.move(-10);
    elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    elevator.setState(ElevatorState::HOLD);
	pros::delay(600);
    clamp.setState(true);
	pros::delay(200);
    elevator.setState(ElevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 
    elevator.goToPinHeight(1);
	moveDrive(0, 100);

    // elevator.setState(ElevatorState::GROUND_PICKUP);
    // moveDrive(-30, 800);
	// leftMotors.move(-10);
	// rightMotors.move(-10);
    // elevator.moveTo(ElevatorConfig::GROUND_CLAMP_ELEV, ElevatorConfig::GROUND_PICKUP_WRIST, false);
    // elevator.setState(ElevatorState::HOLD);
	// pros::delay(600);
    // clamp.setState(true);
	// pros::delay(200);
    // elevator.setState(ElevatorState::PIN_HEIGHT);
	// isNeutralGoal = false; 
    // elevator.goToPinHeight(1);
	// moveDrive(0, 100);
}



inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
        enableColourSort = false;
    }
    if(startingSide == Selection::LEFT) {
        // if(autonSelectionEnum == Selection::LONG_CONTROL) {
        //     // left7Block(); // not long control
        //     left7BlockWing();
        // }
        // if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
        //     // leftLGAndC_CenterLast();
        //     leftLGAndCRush();
        // }
        // if(autonSelectionEnum == Selection::SOLO_AWP) {
        //     midRush();
        // }
        // if(autonSelectionEnum == Selection::NI_SAWP) {
        //     upperMidRush();
        // }
    }
    if(startingSide == Selection::RIGHT) {
        // if(autonSelectionEnum == Selection::LONG_CONTROL) {
        //     // right7Block(); // not long control
        //     // right7BlockWing();
        //     right7Block();
        // }
        // if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
        //     // rightLGAndC_CenterLast();
        //     rightLGAndCRush();
        // }
        // if(autonSelectionEnum == Selection::SOLO_AWP) {
        //     SAWP13();
        // }
        // if(autonSelectionEnum == Selection::NI_SAWP) {
        //     SAWP4Goal();
        // }
    }
}