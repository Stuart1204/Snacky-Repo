#ifndef controls_cpp
#define controls_cpp

#include "globals.hpp"
#include "other_functions.cpp"
#include "auton_functions.cpp"
#include "intake.cpp"
#include "elevator.cpp"
#include "piston.cpp"
#include "pros/colors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstring>
#include <iostream>

inline void intakeControls()
{
    if (controller.get_digital(Buttons::L1))
    {
        if(intakeLift.getState()) {
            intake.setState(IntakeState::INTAKEHIGH);
            elevator.setState(ElevatorState::MATCHLOAD);
        } else {
            intake.setState(IntakeState::INTAKE);
            elevator.setState(ElevatorState::LOAD);
        }
    }
    else if (controller.get_digital(Buttons::L2))
    {
        intake.setState(IntakeState::OUTTAKE);
    } else {
        intake.setState(IntakeState::IDLE);
    }
}
inline void activatePistons()
{
    if (controller.get_digital_new_press(Buttons::Y))
    {
        intakeLift.toggle();
        if(intakeLift.getState()) {
            elevator.setState(ElevatorState::MATCHLOAD);
        } else {
            elevator.setState(ElevatorState::LOAD);
        }
    }
    if (controller.get_digital_new_press(Buttons::X))
    {
        stick.toggle();
    }
    if (controller.get_digital_new_press(Buttons::A))
    {
        clamp.toggle();
    }
}

inline bool toggleFlipActive = false;
inline bool toggleFlipColourSeen = false;

// inline void setToggle() {
//     if (controller.get_digital_new_press(Buttons::B)) {
//         toggleFlipActive = !toggleFlipActive;

//         if (toggleFlipActive) {
//             toggleSensor.set_integration_time(7);
//         } else {
//             toggleSensor.set_integration_time(20);
//             intake.setState(IntakeState::IDLE);
//             return;
//         }
//     }

//     if (!toggleFlipActive) {
//         return;
//     }

//     int detectedColour = checkColour(toggleSensor);
//     bool allianceColourSeen = detectedColour == allianceColour;

//     if (!allianceColourSeen) {
//         if (toggleFlipColourSeen) {
//             toggleFlipActive = false;
//             toggleFlipColourSeen = false;
//             toggleSensor.set_integration_time(20);
//             intake.setState(IntakeState::IDLE);
//         } else {
//             intake.setState(IntakeState::INTAKE);
//         }
//     } else {
//         toggleFlipColourSeen = true;
//         intake.setState(IntakeState::INTAKEHIGH);
//     }

//     pros::screen::print(TEXT_LARGE_CENTER, 1, "seen: %i", allianceColourSeen);
// }

// inline void elevatorControls()
// {
//     if(elevator.getState() == ElevatorState::MANUAL) {
//         if(controller.get_digital(Buttons::R1)) {
//            elevator.setElevatorTarget(100);
//         } else if(controller.get_digital(Buttons::B)) {
//             elevator.setElevatorTarget(-100);
//         } else {
//             elevator.setElevatorTarget(elevator.getElevatorPosition());
//         }
//     }
//     if(controller.get_digital_new_press(Buttons::R1)) {
//         if(elevator.getState() == ElevatorState::LOAD || elevator.getState() == ElevatorState::MATCHLOAD) {
//             elevator.setState(ElevatorState::PICKUP);
//         }
//         if(elevator.getState() == ElevatorState::STOW) {
//             elevator.setState(ElevatorState::MANUAL);
//         }
//     } else if(controller.get_digital(Buttons::R2)) {
//         elevator.setState(ElevatorState::SCORE);
//     } else {
//         // elevator.setElevatorTarget(elevator.getElevatorPosition());
//     }

//     if (controller.get_digital(Buttons::LEFT))
//     {
//         elevator.setState(ElevatorState::STOW);
//     }
//     else if (controller.get_digital(Buttons::RIGHT))
//     {
//         elevator.setState(ElevatorState::LOAD);
//     }
//     else if (controller.get_digital(Buttons::UP))
//     {
//         elevator.setState(ElevatorState::PICKUP);
//     }
//     else if (controller.get_digital(Buttons::DOWN))
//     {
//         elevator.setState(ElevatorState::RESET);
//     }
    // else {
    //     elevator.setState(ElevatorState::REST);
    // }
//}
    // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
	// 		leftElevator.move(127);
	// 		rightElevator.move(-127);
	// 	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
	// 		leftElevator.move(-127);
	// 		rightElevator.move(127);
	// 	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
	// 		leftElevator.move(-127);
	// 		rightElevator.move(-127);
	// 	} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
	// 		leftElevator.move(127);
	// 		rightElevator.move(127);
	// 	} else {
	// 		leftElevator.move(0);
	// 		rightElevator.move(0);
	// 	}

inline void elevatorControls()
{
    int state = elevator.getState();

    // ═══════════════════════════════════════════════════════
    //  R1: INTAKE BUTTON
    //  - From LOAD/MATCHLOAD → initiate PICKUP sequence
    //  - From STOW (complete) → go to MANUAL for fine adjustment
    //  - In MANUAL → drive elevator with held button
    // ═══════════════════════════════════════════════════════

    if(elevator.getState() == ElevatorState::MANUAL) {
        if(controller.get_digital(Buttons::R1)) {
           elevator.setElevatorTarget(100);
        } else if(controller.get_digital(Buttons::B)) {
            elevator.setElevatorTarget(-100);
        } else {
            elevator.setElevatorTarget(elevator.getElevatorPosition());
        }
    }
    if(controller.get_digital_new_press(Buttons::R1)) {
        if(elevator.getState() == ElevatorState::LOAD || elevator.getState() == ElevatorState::MATCHLOAD) {
            elevator.setState(ElevatorState::PICKUP);
        }
        if(elevator.getState() == ElevatorState::STOW) {
            elevator.setState(ElevatorState::MANUAL);
        }
    }

    // ═══════════════════════════════════════════════════════
    //  R2: SCORE — held to score, releases back to REST
    // ═══════════════════════════════════════════════════════
    if (controller.get_digital_new_press(Buttons::R2))
    {
        elevator.setState(ElevatorState::SCORE);
    }

    // ═══════════════════════════════════════════════════════
    //  D-PAD: DIRECT STATE SELECTION
    // ═══════════════════════════════════════════════════════
    if (controller.get_digital_new_press(Buttons::LEFT))
    {
        elevator.setState(ElevatorState::STOW);
    }
    else if (controller.get_digital_new_press(Buttons::RIGHT))
    {
        elevator.setState(ElevatorState::LOAD);
    }
    else if (controller.get_digital_new_press(Buttons::UP))
    {
        elevator.setState(ElevatorState::MATCHLOAD);
    }
    else if (controller.get_digital_new_press(Buttons::DOWN))
    {
        elevator.setState(ElevatorState::RESET);
    }

    // ═══════════════════════════════════════════════════════
    //  ESTOP RECOVERY: Press A to go back to REST from ESTOP
    // ═══════════════════════════════════════════════════════
    if (state == ElevatorState::ESTOP)
    {
        controller.rumble("-");
        elevator.setState(ElevatorState::REST);
    }
}


inline void drivetrainControls()
{
    leftMotors.set_brake_mode(MotorConfigs::BRAKE);
    rightMotors.set_brake_mode(MotorConfigs::BRAKE);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (abs(leftY) < 20)
    {
        leftY = 0;
    }
    if (abs(rightX) < 20)
    {
        rightX = 0;
    }
    chassis.arcade(leftY, rightX, false, 0.5);
}

inline void subsystemControls()
{
    // setToggle();
    if (!toggleFlipActive) {
        intakeControls();
    }
    elevatorControls();
    activatePistons();
}

inline void displayData()
{
    float frontCurrentDistance = (frontDistSensor.get_distance() * MM_TO_INCHES) + FRONT_DIST_OFFSET;
    float leftCurrentDistance = (leftDistSensor.get_distance() * MM_TO_INCHES) + LEFT_DIST_OFFSET;
    float rightCurrentDistance = (rightDistSensor.get_distance() * MM_TO_INCHES) + RIGHT_DIST_OFFSET;
    // display lemlib pose
    lemlib::Pose pose = chassis.getPose();
    // pros::screen::print(TEXT_LARGE_CENTER, 1, "X: %f", pose.x);
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "Y: %f", pose.y);
    // pros::screen::print(TEXT_LARGE_CENTER, 5, "Theta: %f", pose.theta);
    // pros::screen::print(TEXT_LARGE_CENTER, 7, "Colour: %i", checkColour(toggleSensor));
    // pros::screen::print(TEXT_LARGE_CENTER, 9, "dist: %f", rightCurrentDistance);

    pros::screen::print(TEXT_LARGE_CENTER, 1, "elevatorPos: %f", elevator.getElevatorPosition());
    pros::screen::print(TEXT_LARGE_CENTER, 3, "wristPos: %f", elevator.getWristPosition());
    pros::screen::print(TEXT_LARGE_CENTER, 9, "elevatorTarget: %f", elevator.getElevatorTarget());
    // pros::screen::print(TEXT_LARGE_CENTER, 7, "ele: %f", (leftElevator.get_position() - rightElevator.get_position()) / 2.0);
    pros::screen::print(TEXT_LARGE_CENTER, 5, "leftPower: %i", elevator.getLeftPower());
    pros::screen::print(TEXT_LARGE_CENTER, 7, "rightPower: %i", elevator.getRightPower());


}

class button
{
public:
    double x;
    double y;
    double x2;
    double y2;
    pros::Color colour;
    std::string text;
    const float characterToOffset = 8.75;
    int textOffset;
    bool pressed = false;
    button(double x, double y, double x2, double y2, pros::Color colour, const char *text) : x(x), y(y), x2(x2), y2(y2),
                                                                                             colour(colour), text(text)
    {

        pros::screen::set_pen(colour);
        pros::screen::fill_rect(x, y, x2, y2);
        pros::screen::set_pen(pros::Color::black);
        pros::screen::draw_rect(x, y, x2, y2);
        textOffset = std::strlen(text) * characterToOffset;
        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_LARGE, (x + x2) / 2 - textOffset, (y + y2) / 2, text);
    }
    bool isPressed()
    {
        pros::screen_touch_status_s_t currentTouch = pros::screen::touch_status();

        // Check if touch is within button bounds and is a press
        if (currentTouch.touch_status == pros::E_TOUCH_PRESSED &&
            currentTouch.x >= x && currentTouch.x <= x2 &&
            currentTouch.y >= y && currentTouch.y <= y2)
        {
            return true;
        }
        return false;
    }
};

inline void clearScreen()
{
    pros::screen::set_pen(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "Loading...");
    pros::screen::set_pen(pros::Color::black);
    pros::delay(300);
    pros::screen::erase();
}

// inline void autonSelection() {
//     button skillsButton(20, 15, 225, 220, pros::Color::fuchsia, "Skills");
//     button matchButton(250, 15, 455, 220, pros::Color::alice_blue, "Match");
//     // selecting between skills and match
//     while(true) {
//         if(skillsButton.isPressed()) {
//             autonSelectionEnum = Selection::SKILLS;
//             break;
//         }
//         if(matchButton.isPressed()) {
//             autonSelectionEnum = Selection::MATCH;
//             break;
//         }
//         pros::delay(20);
//     }
//     clearScreen();
//     if(autonSelectionEnum == Selection::SKILLS) {
//         button skillsScreen(0, 0, 480, 200, pros::Color::fuchsia, "Skills Selected!");
//     }
//     if(autonSelectionEnum == Selection::MATCH) {
//         button redAllianceButton(20, 15, 225, 220, pros::Color::orange_red, "Red");
//         button blueAllianceButton(250, 15, 455, 220, pros::Color::light_blue, "Blue");
//         while(true) {
//             if(redAllianceButton.isPressed()) {
//                 allianceColour = AllianceColour::RED;
//                 break;
//             }
//             if(blueAllianceButton.isPressed()) {
//                 allianceColour = AllianceColour::BLUE;
//                 break;
//             }
//             pros::delay(20);
//         }
//         clearScreen();

//         button leftSideButton(20, 15, 225, 180, pros::Color::sky_blue, "Left");
//         button rightSideButton(250, 15, 455, 180, pros::Color::sky_blue, "Right");
//         if(allianceColour == AllianceColour::RED) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red");
//         }
//         if(allianceColour == AllianceColour::BLUE) {
//             pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue");
//         }

//         while(true) {
//             if(leftSideButton.isPressed()) {
//                 startingSide = Selection::LEFT;
//                 break;
//             }
//             if(rightSideButton.isPressed()) {
//                 startingSide = Selection::RIGHT;
//                 break;
//             }
//             pros::delay(20);
//         }
//         clearScreen();

//         button longControlButton(0, 15, 120, 180, pros::Color::sky_blue, "LG");
//         button longAndCenterButton(120, 15, 240, 180, pros::Color::sky_blue, "LG+C");
//         button SAWPButton(240, 15, 360, 180, pros::Color::sky_blue, "SAWP or Both Mid");
//         button NISAWPButton(360, 15, 480, 180, pros::Color::sky_blue, "4-Goal or UpMid Rush");

//         if(allianceColour == AllianceColour::RED) {
//             if(startingSide == Selection::LEFT) {
//                 pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red, Left");
//             }
//             if(startingSide == Selection::RIGHT) {
//                 pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red, Right");
//             }
//         }
//         if(allianceColour == AllianceColour::BLUE) {
//             if(startingSide == Selection::LEFT) {
//                 pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue, Left");
//             }
//             if(startingSide == Selection::RIGHT) {
//                 pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue, Right");
//             }
//         }

//         while(true) {
//             if(longControlButton.isPressed()) {
//                 autonSelectionEnum = Selection::LONG_CONTROL;
//                 break;
//             }
//             if(longAndCenterButton.isPressed()) {
//                 autonSelectionEnum = Selection::LONG_AND_CENTER;
//                 break;
//             }
//             if(SAWPButton.isPressed()) {
//                 autonSelectionEnum = Selection::SOLO_AWP;
//                 break;
//             }
//             if(NISAWPButton.isPressed()) {
//                 autonSelectionEnum = Selection::NI_SAWP;
//                 break;
//             }
//             pros::delay(20);
//         }
//         clearScreen();

//         if(allianceColour == AllianceColour::RED) {
//             if(startingSide == Selection::LEFT) {
//                 if(autonSelectionEnum == Selection::LONG_CONTROL) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, LG");
//                 }
//                 if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, LG+C");
//                 }
//                 if(autonSelectionEnum == Selection::SOLO_AWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "both mid rush");
//                 }
//                 if(autonSelectionEnum == Selection::NI_SAWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "up mid rush");
//                 }
//             }
//             if(startingSide == Selection::RIGHT) {
//                 if(autonSelectionEnum == Selection::LONG_CONTROL) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, LG");
//                 }
//                 if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, LG+C");
//                 }
//                 if(autonSelectionEnum == Selection::SOLO_AWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, SAWP");
//                 }
//                 if(autonSelectionEnum == Selection::NI_SAWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, NISAWP");
//                 }
//             }
//         }
//         if(allianceColour == AllianceColour::BLUE) {
//             if(startingSide == Selection::LEFT) {
//                 if(autonSelectionEnum == Selection::LONG_CONTROL) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, LG");
//                 }
//                 if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, LG+C");
//                 }
//                 if(autonSelectionEnum == Selection::SOLO_AWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "both mid rush");
//                 }
//                 if(autonSelectionEnum == Selection::NI_SAWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "up mid rush");
//                 }
//             }
//             if(startingSide == Selection::RIGHT) {
//                 if(autonSelectionEnum == Selection::LONG_CONTROL) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, LG");
//                 }
//                 if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, LG+C");
//                 }
//                 if(autonSelectionEnum == Selection::SOLO_AWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, SAWP");
//                 }
//                 if(autonSelectionEnum == Selection::NI_SAWP) {
//                     button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, NISAWP");
//                 }
//             }
//         }
//     }
// }

#endif