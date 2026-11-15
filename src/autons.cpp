#include "globals.hpp"
#include "intakevator.cpp"
#include "piston.cpp"
#include "auton_functions.cpp"
#include "pros/rtos.hpp"
#include <cstdint>

inline void skillsAuton() {
    localization.resetPose(-60.5, 9, -128);
    isNeutralGoal = false;
    isAutoReset = true;
    isAutonSkills = true;
    intakevator.setState(IntakevatorState::PICKUP);
    moveDrive(105, 650);
    turnToPoint(-55, -21, false);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(0);
    moveDrive(-70, 500);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    goalReset();
    pros::delay(350);
    moveToPoint(-48, -5, true, false);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-28, -23, false, true, true);
    leftMotors.move(-12);
	rightMotors.move(-12);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV - .25, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);

    //PICKUS UP 1st ground Pin

    // intakevator.goToPinHeight(1);
    intakevator.moveTo(IntakevatorConfig::PIN_1_ELEV-2.52-.5, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
    pros::delay(200);
    moveToPoint(-42, -24, false, true);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(450);
    goalReset();
    isAutoReset = true;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 350);
    pros::delay(1250);
    intakevator.resetEncoders();
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-44, -44, false, true, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(500);
    clamp.setState(true);
	pros::delay(200);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(2);
    //since elevator is off this is the right height for pin heihgt 3 alliance goal
    pros::delay(200);
    moveToPoint(-48, -27, false, true, true);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    intakevator.setState(IntakevatorState::SCORE);
    goalReset();
    pros::delay(350);
    matchloadMode = true;
    intakevator.setState(IntakevatorState::IDLE);
    hood.setState(true);

    skillsAquireAllianceMatchload();
    intakevator.goToPinHeight(3);
    skillsScoreOnAllianceGoal();

    // skillsAquireAllianceMatchload();
    // intakevator.goToPinHeight(4);
    // skillsScoreOnAllianceGoal();
    intakevator.setState(IntakevatorState::IDLE);

    moveToPoint(-48, -45, true, false, true);
    matchloadMode = false;
    intakeLift.setState(false);
    hood.setState(false);
    moveToPoint(-35, -31, false, false, true, 80);
    moveToPoint(-35, -12, false, false, true, 80);
    moveToPoint(-30,-1, false, false, true, 70);
    intakeLift.setState(true);
    moveToPoint(-30, 12, true, true);
    intakeLift.setState(false);
    intakevator.setState(IntakevatorState::INTAKE);
    leftMotors.move(10);
    rightMotors.move(10);
    pros::delay(1000);
    turnToPoint(-56.5, 2, false);
    chassis.moveToPoint(-56.5, 2, 1000, {.forwards = false});
    chassis.waitUntil(6);
    isNeutralGoal = true;
    

    // moveToPoint(-48, 2, false, true);
    turnToHeading(180, true);
    intakevator.setState(IntakevatorState::PICKUP);
    pros::delay(650);
    moveDrive(-60, 500);
    intakevator.goToPinHeight(0);
    pros::delay(350);
    intakevator.setState(IntakevatorState::SCORE);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(400);
    goalReset();
    moveToPoint(-60, 1, true, false, true);
    moveToPoint(-60, 40, false, true, true);
    turnToHeading(180, true);
    rightResetPos();
    moveToPoint(-45, 60, false, true, true);
    matchloadMode = true;
    intakevator.setState(IntakevatorState::IDLE);
    isAutoReset = true;
    hood.setState(true);
    turnToHeading(-90, false);
    rightResetPos();
    moveToPoint(-58, 56, true, true);
    intakevator.setState(IntakevatorState::IDLE);
    intakevator.setState(IntakevatorState::PICKUP);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    turnToHeading(-90, true);
    moveDrive(30, 750);
	moveDrive(-12, 150);
    isNeutralGoal = true;
    uint32_t startTime = pros::millis();
    while(intakevator.getState() != IntakevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2750) {
            break;
        }
    }
    isNeutralGoal = true;
    intakevator.goToPinHeight(1);
    // skillsScoreOnNeutralGoal();
    rightResetPos();
    moveDrive(-50, 300);
    moveToPoint(-49, 33, false, true, true, 70);
    moveDrive(-35, 500);
    leftMotors.move(-30);
    rightMotors.move(-30);
    intakevator.setState(IntakevatorState::SCORE);

    goalReset();
    pros::delay(350);

    skillsAcquireNeutralMatchload();

    intakevator.goToPinHeight(2);
    skillsScoreOnNeutralGoal();
    skillsAcquireNeutralMatchload();

    intakevator.goToPinHeight(3);
    skillsScoreOnNeutralGoal();
    // isDropDownIntake = false;
    
    // intakevator.setState(IntakevatorState::IDLE);
    // isNeutralGoal = false;
    // matchloadMode = false;
    // goalReset();
	// turnToHeading(0, true);
    moveDrive(50, 300);
    moveDrive(10, 50);
    // turnToPoint(-27, 33, true);
    // chassis.moveToPoint(-27, 33, 1000, {.maxSpeed = 60,.minSpeed = 10, .earlyExitRange = 5});
    // chassis.waitUntil(7);
    // intakeLift.setState(true);
    // chassis.waitUntilDone();
	// leftMotors.move(50);
	// rightMotors.move(-5);
	// pros::delay(150);
	// intakeLift.setState(false);
    // isDropDownIntake = true;
	// intakevator.setState(IntakevatorState::INTAKE);
	// pros::delay(550);
	// moveDrive(5, 100);
	// pros::delay(700);
	// turnToPoint(-2, 29, false);
	// chassis.moveToPoint(-2, 29, 1000, {.forwards = false, .earlyExitRange = 5});
	// chassis.waitUntil(7);
	// intakevator.setState(IntakevatorState::PICKUP);
    // chassis.moveToPoint(0, 7, 1000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 3});
    // chassis.waitUntil(10);
    // intakevator.moveTo(3, IntakevatorConfig::WRIST_EXTENDED, false);
    // intakevator.setState(IntakevatorState::HOLD);
    // leftMotors.move(-35);
    // rightMotors.move(-35);
    // pros::delay(450);
    // isAutoReset = false;
    // intakevator.setState(IntakevatorState::SCORE);
    // pros::delay(350);
    // goalReset();
    // moveDrive(50, 300);
	// intakevator.setState(IntakevatorState::GROUND_PICKUP);
	// moveDrive(10, 50);
    // moveToPoint(19, 21, false, true);
    // leftMotors.move(-15);
	// rightMotors.move(-15);
    // intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV - 1, IntakevatorConfig::WRIST_EXTENDED, true);
    // intakevator.setState(IntakevatorState::HOLD);
	// pros::delay(500);
    // clamp.setState(true);
	// pros::delay(200);
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);
	// isNeutralGoal = false;
	// intakevator.moveTo(10.5, IntakevatorConfig::WRIST_EXTENDED, false);
    // intakevator.setState(IntakevatorState::HOLD);
    // pros::delay(200);
    // moveToPoint(6, 4, false, true, true);
    // leftMotors.move(-55);
    // rightMotors.move(-55);
    // pros::delay(500);
    // intakevator.setState(IntakevatorState::SCORE);
    moveToPoint(0, 0, true, true);
}

inline void skillsDriverMacro() {
    localization.resetPose(-60.5, 9, -128);
    isNeutralGoal = false;
    isAutoReset = true;
    isAutonSkills = true;
    intakevator.setState(IntakevatorState::PICKUP);
    moveDrive(105, 650);
    turnToPoint(-55, -21, false);
    matchloadMode = true;
    intakevator.setState(IntakevatorState::IDLE);
    hood.setState(true);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(0);
    moveDrive(-70, 500);
    leftMotors.move(-35);
    rightMotors.move(-35);
    pros::delay(300);
    intakevator.setState(IntakevatorState::SCORE);
    if (skillsTask != nullptr) {
        skillsTask->suspend();
        isSkillsMacro = false;
	}
    // goalReset();
    // pros::delay(350);
    // moveToPoint(-48, -5, true, false, true);
    // intakevator.setState(IntakevatorState::GROUND_PICKUP);
    // moveToPoint(-28, -23, false, true, true);
    // leftMotors.move(-12);
	// rightMotors.move(-12);
    // intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV - .25, IntakevatorConfig::WRIST_EXTENDED, false);
    // intakevator.setState(IntakevatorState::HOLD);
	// pros::delay(500);
    // clamp.setState(true);
	// pros::delay(200);
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);

    // //PICKUS UP 1st ground Pin

    // // intakevator.goToPinHeight(1);
    // intakevator.moveTo(IntakevatorConfig::PIN_1_ELEV-2.52-.5, IntakevatorConfig::WRIST_EXTENDED, false);
    // intakevator.setState(IntakevatorState::HOLD);
    // pros::delay(200);
    // moveToPoint(-42, -24, false, true);
    // leftMotors.move(-35);
    // rightMotors.move(-35);
    // pros::delay(450);
    // goalReset();
    // isAutoReset = true;
    // intakevator.setState(IntakevatorState::SCORE);
    // pros::delay(300);
    // moveDrive(50, 350);
    // pros::delay(1250);
    // intakevator.resetEncoders();
    // intakevator.setState(IntakevatorState::GROUND_PICKUP);
    // moveToPoint(-44, -44, false, true, true);
    // leftMotors.move(-15);
	// rightMotors.move(-15);
    // intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    // intakevator.setState(IntakevatorState::HOLD);
	// pros::delay(500);
    // clamp.setState(true);
	// pros::delay(200);
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);
    // intakevator.goToPinHeight(2);
    // //since elevator is off this is the right height for pin heihgt 3 alliance goal
    // pros::delay(200);
    // moveToPoint(-48, -27, false, true, true);
    // leftMotors.move(-35);
    // rightMotors.move(-35);
    // pros::delay(300);
    // intakevator.setState(IntakevatorState::SCORE);
    // goalReset();
    // pros::delay(350);
    // matchloadMode = true;
    // intakeLift.setState(true);
    // hood.setState(true);
    // leftMotors.move(50);
    // rightMotors.move(50);
    // if (skillsTask != nullptr) {
    //     skillsTask->suspend();
    //     isSkillsMacro = false;
	// }
}

inline void autonFirstPartLeft() {
    localization.resetPose(-60.5, 9, -128);
    isNeutralGoal = false;
    intakevator.setState(IntakevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(-25, 300, {.minSpeed = 80});
    chassis.moveToPoint(-62, -10, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(0);
    matchloadMode = true;
    intakevator.setState(IntakevatorState::IDLE);
    hood.setState(true);
    chassis.swingToHeading(-93.5, DriveSide::RIGHT, 700, {.minSpeed = 90, .earlyExitRange = 8});
    chassis.waitUntilDone();
    moveDrive(0, 200);
    leftMotors.move(-10);
    rightMotors.move(-10);
    intakevator.setState(IntakevatorState::SCORE);
    goalReset();
    pros::delay(300);
    isFastReset = true;
    intakevator.setState(IntakevatorState::RESET);
    turnToHeading(-90);
    intakevator.setState(IntakevatorState::IDLE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	moveDrive(45, 400);
	moveDrive(-20, 200);
    uint32_t startTime = pros::millis();
    while(intakevator.getState() != IntakevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 1800) {
            break;
        }
        if(pros::millis() - startTime > 1500) {
            hood.setState(false);
        }
    }
    if(intakevator.getState() != IntakevatorState::PIN_HEIGHT) {
        intakevator.setState(IntakevatorState::PICKUP);
    }
    isFastReset = false;
    turnToHeading(-80, true);
    moveDrive(-60, 500);
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    goalReset();
    pros::delay(350);
    intakevator.setState(IntakevatorState::RESET);
    chassis.moveToPoint(-63, -10, 700, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-63, -30, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
  //  chassis.swingToHeading(-153, DriveSide::LEFT,  1000, {.minSpeed = 80, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-50, -46, false, true, true);
    leftMotors.move(-10);
	rightMotors.move(-10);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 

    // PICKS UP FIRST GROUND PIN
}
inline void autonFirstPartRight() {
    localization.resetPose(9, -60.5, 218);
    isNeutralGoal = false;
    intakevator.setState(IntakevatorState::PICKUP);
    moveDrive(105, 650);
    chassis.turnToHeading(135, 450, {.minSpeed = 80});
    chassis.moveToPoint(-10, -61.75, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    chassis.waitUntil(6);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(0);
    matchloadMode = true;
    intakevator.setState(IntakevatorState::IDLE);
    hood.setState(true);
    chassis.swingToHeading(180, DriveSide::LEFT, 700, {.minSpeed = 90, .earlyExitRange = 10});
    chassis.waitUntilDone();
    intakevator.moveTo(IntakevatorConfig::PIN_0_ELEV - IntakevatorConfig::ALLIANCE_OFFSET, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
    moveDrive(0, 200);
    intakevator.setState(IntakevatorState::SCORE);
    moveDrive(-40, 300);
    isFastReset = true;
	goalReset();
    intakevator.setState(IntakevatorState::RESET);
	goalJiggle(180, 300);
    goalJiggle(180, 500);
    // pros::delay(300);
    intakevator.setState(IntakevatorState::IDLE);
    leftMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	rightMotors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	// moveDrive(25, 550);
	// moveDrive(-20, 200);
    driveDistance(4.25);
    uint32_t startTime = pros::millis();
    while(intakevator.getState() != IntakevatorState::PIN_HEIGHT) {
        pros::delay(10);
        if(pros::millis() - startTime > 2000) {
            break;
        }
    }
    isFastReset = true;
    moveDrive(-60, 500);
    isAutoReset = true;
    intakevator.setState(IntakevatorState::SCORE);
    intakevator.setState(IntakevatorState::OUTTAKE);
    goalReset();
    pros::delay(350);
    // intakevator.setState(IntakevatorState::RESET);
    chassis.moveToPoint(-10, -63, 700, {.minSpeed = 30, .earlyExitRange = 3});
    chassis.moveToPoint(-30, -63, 700, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    intakevator.resetEncoders();
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    isFastReset = false;
    moveToPoint(-47, -49, false, true, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
    intakevator.setState(IntakevatorState::IDLE);
    leftMotors.move(0);
    rightMotors.move(0);
	pros::delay(200);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
	isNeutralGoal = false; 

    // PICKS UP FIRST GROUND PIN
}
inline void SAWP() {

}

inline void TAWPLeft() { // teamate has to score one pin and get toggle
    autonFirstPartLeft();
    // PICKS UP FIRST GROUND PIN

    intakevator.goToPinHeight(1);
    pros::delay(200);
    chassis.moveToPoint(-27, -47, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-25, -28, false, true, true);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV-.4, IntakevatorConfig::WRIST_EXTENDED, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);
	isNeutralGoal = true; 
    // intakevator.goToPinHeight(1);
    isAutoReset = true;
    intakevator.moveTo(IntakevatorConfig::PIN_1_ELEV - 1.5, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
    pros::delay(200);
    chassis.turnToPoint(-42, 18, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(-42, 18, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    intakevator.setState(IntakevatorState::SCORE);
}

inline void TAWPRight() {
    autonFirstPartRight();

    intakevator.goToPinHeight(1);
    pros::delay(200);
    chassis.moveToPoint(-48, -27, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-28, -24.5, false, true, true);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);
	// isNeutralGoal = true; 
    // intakevator.goToPinHeight(1);
    intakevator.moveTo(IntakevatorConfig::PIN_1_ELEV - .5, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
    isAutoReset = true;
    // pros::delay(200);
    chassis.turnToPoint(20, -42, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(20, -42, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    pros::delay(250);
    intakevator.setState(IntakevatorState::SCORE);
}

inline void elimsAllianceGoalLeft() {
    autonFirstPartLeft();

    intakevator.goToPinHeight(2);
    pros::delay(200);
    chassis.turnToPoint(-41, -27, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-41, -27, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-29, -24, false, true, true);
    intakevator.moveTo(-5, IntakevatorConfig::WRIST_EXTENDED, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    isNeutralGoal = false;
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(3);
    // since elevator is off this is the right height for pin heihgt 3 alliance goal
    isAutoReset = true;
    pros::delay(200);
    chassis.turnToPoint(-45, -21, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-45, -21, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    intakevator.setState(IntakevatorState::SCORE);

}

inline void iceAutoRight() {
    autonFirstPartRight();
    isNeutralGoal = true;
    intakevator.goToPinHeight(1);
    pros::delay(200);
    moveToPoint(-24, -30, true, true);
    moveToPoint(19, -45, false, true);
    leftMotors.move(-45);
    rightMotors.move(-45);
    pros::delay(350);
    intakevator.setState(IntakevatorState::SCORE);
 }  

inline void elimsAutoConsistentRight() {
    localization.resetPose(9, -60.5, 218);
    isNeutralGoal = false;
    intakevator.setState(IntakevatorState::PICKUP);
    moveDrive(105, 650);
    // chassis.turnToHeading(145, 450, {.minSpeed = 50});
    // chassis.waitUntilDone();
    // moveDrive(-50, 300);
    // chassis.moveToPoint(-10, -53, 1500, {.forwards = false, .minSpeed = 90, .earlyExitRange = 5});
    moveToPoint(-10, -55, false, true);
    chassis.waitUntil(6);
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(0);
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(700);
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveToPoint(-6, -60.25, true, true);
    moveToPoint(-36, -60.25, false, true);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    pros::delay(500);
    moveToPoint(-43, -51, false, true);
    leftMotors.move(-15);
	rightMotors.move(-15);
    pros::delay(200);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
    intakevator.setState(IntakevatorState::IDLE);
    leftMotors.move(0);
    rightMotors.move(0);
	pros::delay(500);
    isNeutralGoal = false; 
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(1);
    turnToPoint(-24, -48, false);
    moveDrive(-45, 900);
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(600);
    moveDrive(40, 500);
    turnToPoint(-6, -31, false);
    pros::delay(500);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-6, -31, false, false, true);
    leftMotors.move(-20);
	rightMotors.move(-20);
    pros::delay(200);
    intakevator.moveTo(IntakevatorConfig::GROUND_CLAMP_ELEV, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
    intakevator.setState(IntakevatorState::IDLE);
    isNeutralGoal = true; 
    intakevator.setState(IntakevatorState::PIN_HEIGHT);
    intakevator.goToPinHeight(1);
    moveDrive(-30, 300);
}

inline void elimsAllianceGoalRight() {
    
    autonFirstPartRight();

    intakevator.goToPinHeight(2);
    pros::delay(200);
    chassis.turnToPoint(-27, -41, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-27, -41, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-24, -29, false, true, true);
    intakevator.moveTo(-5, IntakevatorConfig::WRIST_EXTENDED, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    isNeutralGoal = false;
    // intakevator.setState(IntakevatorState::PIN_HEIGHT);
    // intakevator.goToPinHeight(2);
    // since elevator is off this is the right height for pin heihgt 3 alliance goal
    intakevator.moveTo(IntakevatorConfig::PIN_3_ELEV - 4.75, IntakevatorConfig::WRIST_EXTENDED, false);
    isAutoReset = true;
    pros::delay(200);
    chassis.turnToPoint(-19, -45, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-19, -45, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-40);
    rightMotors.move(-40);
    intakevator.setState(IntakevatorState::SCORE);
}


inline void hybridRight() {
    autonFirstPartRight();

    intakevator.goToPinHeight(2);
    pros::delay(200);
    chassis.turnToPoint(-27, -41, 800, {.forwards = false, .minSpeed = 40, .earlyExitRange = 10});
    chassis.moveToPoint(-27, -41, 700, {.forwards = false, .earlyExitRange = 4});
    chassis.waitUntilDone();
    leftMotors.move(-15);
    rightMotors.move(-15);
    pros::delay(300);
    goalReset();
    isAutoReset = false;
    intakevator.setState(IntakevatorState::SCORE);
    pros::delay(300);
    moveDrive(50, 250);
    intakevator.setState(IntakevatorState::GROUND_PICKUP);
    moveToPoint(-24, -29, false, true, true);
    intakevator.moveTo(-5, IntakevatorConfig::WRIST_EXTENDED, false);
    leftMotors.move(-15);
	rightMotors.move(-15);
    intakevator.setState(IntakevatorState::HOLD);
	pros::delay(400);
    clamp.setState(true);
	pros::delay(200);
    isNeutralGoal = false;
    intakevator.moveTo(IntakevatorConfig::PIN_1_ELEV - .5, IntakevatorConfig::WRIST_EXTENDED, false);
    intakevator.setState(IntakevatorState::HOLD);
    isAutoReset = true;
    // pros::delay(200);
    chassis.turnToPoint(20, -42, 300, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.moveToPoint(20, -42, 1500, {.forwards = false, .minSpeed = 30, .earlyExitRange = 5});
    chassis.waitUntilDone();
    leftMotors.move(-20);
    rightMotors.move(-20);
    pros::delay(250);
    intakevator.setState(IntakevatorState::SCORE);
}



inline void runSelectedAuton() {
    if(autonSelectionEnum == Selection::SKILLS) {
        skillsAuton();
        allianceColour = AllianceColour::RED;
    }
    if(startingSide == Selection::LEFT) {
        if(autonSelectionEnum == Selection::TAWP) {
            TAWPLeft();
        }
        if(autonSelectionEnum == Selection::ELIMS) {
            elimsAllianceGoalLeft();
        }
        if(autonSelectionEnum == Selection::CORNER) {

        }
        if(autonSelectionEnum == Selection::NOTHING) {

        }
    }
    if(startingSide == Selection::RIGHT) {
        if(autonSelectionEnum == Selection::TAWP) {
            TAWPRight();
        }
        if(autonSelectionEnum == Selection::ELIMS) {
            // rightLGAndC_CenterLast();
            elimsAllianceGoalRight();
        }
        if(autonSelectionEnum == Selection::CORNER) {

        }
        if(autonSelectionEnum == Selection::NOTHING) {

        }
    }
}