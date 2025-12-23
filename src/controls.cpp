#ifndef controls_cpp
#define controls_cpp

#include "globals.hpp"
#include "other_functions.cpp"
#include "intake.cpp"
#include "piston.cpp"
#include "pros/colors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstring>
#include <iostream>

inline void intakeControls() {
    if(controller.get_digital(Buttons::R1) && controller.get_digital(Buttons::L1)) {
        intake.setState(IntakeState::SCOREHIGHSLOW);
    } else if(controller.get_digital(Buttons::L1)) {
        intake.setState(IntakeState::INTAKE);
    } else if(controller.get_digital(Buttons::L2)) {
        // scoreLow scores low goal when intake is up
        if(intakeLift.getState() == true) {
            intake.setState(IntakeState::SCORELOW);
        } else {
            intake.setState(IntakeState::OUTTAKE);
        }
    } else if(controller.get_digital(Buttons::R1)) {
        intake.setState(IntakeState::SCOREHIGH);
    } else if(controller.get_digital(Buttons::R2)) {
        intake.setState(IntakeState::SCOREMID);
    } else {
        if(intake.getState() != IntakeState::PARK) {
            intake.setState(IntakeState::IDLE);
        }
    }

    // uses the same park button to exit off park
    if(controller.get_digital_new_press(Buttons::B)) {
        if(park.getState() == true) {
            park.setState(false);
        } else {
            intake.setState(IntakeState::PARK);
        }
    }

    if(controller.get_digital_new_press(Buttons::RIGHT)) {
        enableColourSort = !enableColourSort;
        if(enableColourSort) {
            controller.rumble("...");
        } else {
            controller.rumble(".");
        }
    }

    if(controller.get_digital_new_press(Buttons::LEFT)) {
        slowLowGoal = !slowLowGoal;
        if(slowLowGoal) {
           controller.rumble(".."); 
        } else {
            controller.rumble(".");
        }
    }
}

inline void drivetrainParkControl() {
    if(controller.get_digital(Buttons::DOWN)) {
        leftMotors.move(-45);
        rightMotors.move(-45);
    }
}

inline void activatePistons() {
    if (controller.get_digital_new_press(Buttons::X)) {
        wing.toggle();
    }
    if (controller.get_digital_new_press(Buttons::A)) {
        tongue.toggle();
    }
    // double park macro uses activate park to extened pistons
    if(activatePark) {
        park.setState(true);
        activatePark = false;
    }
    if (controller.get_digital_new_press(Buttons::Y)) {
        intakeLift.toggle();
    }
    if (controller.get_digital_new_press(Buttons::UP)) {
        sunroof.toggle();
    }
}

inline int matchloadShakeCounter = 0;
inline bool matchloadShakeDirection = false;

inline void subsystemControls() {
    intakeControls(); 
    activatePistons();
    //drivetrainParkControl();
    countBlocks(parkSensor);
}

inline void drivetrainControls() {
    leftMotors.set_brake_mode(MotorConfigs::BRAKE);
	rightMotors.set_brake_mode(MotorConfigs::BRAKE);
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if(!matchloadShake) {
        chassis.arcade(leftY, rightX, false, 0.5);
    }
}

inline void displayData() {
    float frontCurrentDistance = (frontDistSensor.get_distance() * MM_TO_INCHES) + FRONT_DIST_OFFSET;
    float leftCurrentDistance = (leftDistSensor.get_distance() * MM_TO_INCHES) + LEFT_DIST_OFFSET;
    float rightCurrentDistance = (rightDistSensor.get_distance() * MM_TO_INCHES) + RIGHT_DIST_OFFSET;
    // display lemlib pose
    lemlib::Pose pose = chassis.getPose();
    pros::screen::print(TEXT_LARGE_CENTER, 9, "dist: %f", frontDistSensor.get_distance() * MM_TO_INCHES);

    pros::screen::print(TEXT_LARGE_CENTER, 1, "x: %f", pose.x);
    pros::screen::print(TEXT_LARGE_CENTER, 3, "y: %f", pose.y);
    pros::screen::print(TEXT_LARGE_CENTER, 5, "heading: %f", pose.theta);
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
    button(double x, double y, double x2, double y2, pros::Color colour, const char* text) : x(x), y(y), x2(x2), y2(y2), 
        colour(colour), text(text) {
        
        pros::screen::set_pen(colour);
	    pros::screen::fill_rect(x,y,x2,y2);
        pros::screen::set_pen(pros::Color::black);
        pros::screen::draw_rect(x, y, x2, y2);
        textOffset = std::strlen(text) * characterToOffset;
        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_LARGE, (x + x2) / 2 - textOffset, (y + y2) / 2, text);
    }
    bool isPressed() {
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

inline void clearScreen() {
    pros::screen::set_pen(pros::Color::black);
    pros::screen::erase();
    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 5, "Loading...");
    pros::screen::set_pen(pros::Color::black);
    pros::delay(300);
    pros::screen::erase();
}

inline void autonSelection() {
    button skillsButton(20, 15, 225, 220, pros::Color::fuchsia, "Skills");
    button matchButton(250, 15, 455, 220, pros::Color::alice_blue, "Match");
    // selecting between skills and match
    while(true) {
        if(skillsButton.isPressed()) {
            autonSelectionEnum = Selection::SKILLS;
            break;
        }
        if(matchButton.isPressed()) {
            autonSelectionEnum = Selection::MATCH;
            break;
        }
        pros::delay(20);
    }
    clearScreen();
    if(autonSelectionEnum == Selection::SKILLS) {
        button skillsScreen(0, 0, 480, 200, pros::Color::fuchsia, "Skills Selected!");
    }
    if(autonSelectionEnum == Selection::MATCH) {
        button redAllianceButton(20, 15, 225, 220, pros::Color::orange_red, "Red");
        button blueAllianceButton(250, 15, 455, 220, pros::Color::light_blue, "Blue");
        while(true) {
            if(redAllianceButton.isPressed()) {
                allianceColour = AllianceColour::RED;
                break;
            }
            if(blueAllianceButton.isPressed()) {
                allianceColour = AllianceColour::BLUE;
                break;
            }
            pros::delay(20);
        }
        clearScreen();

        button leftSideButton(20, 15, 225, 180, pros::Color::sky_blue, "Left");
        button rightSideButton(250, 15, 455, 180, pros::Color::sky_blue, "Right");
        if(allianceColour == AllianceColour::RED) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red");
        } 
        if(allianceColour == AllianceColour::BLUE) {
            pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue");
        }
        
        while(true) {
            if(leftSideButton.isPressed()) {
                startingSide = Selection::LEFT;
                break;
            }
            if(rightSideButton.isPressed()) {
                startingSide = Selection::RIGHT;
                break;
            }
            pros::delay(20);
        }
        clearScreen();

        button longControlButton(0, 15, 120, 180, pros::Color::sky_blue, "LG");
        button longAndCenterButton(120, 15, 240, 180, pros::Color::sky_blue, "LG+C");
        button SAWPButton(240, 15, 360, 180, pros::Color::sky_blue, "SAWP");
        button NISAWPButton(360, 15, 480, 180, pros::Color::sky_blue, "NISAWP");

        if(allianceColour == AllianceColour::RED) {
            if(startingSide == Selection::LEFT) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red, Left");
            }
            if(startingSide == Selection::RIGHT) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Red, Right");
            }
        }
        if(allianceColour == AllianceColour::BLUE) {    
            if(startingSide == Selection::LEFT) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue, Left");
            }
            if(startingSide == Selection::RIGHT) {
                pros::screen::print(pros::E_TEXT_MEDIUM, 10, "Selected: Blue, Right");
            }
        }

        while(true) {
            if(longControlButton.isPressed()) {
                autonSelectionEnum = Selection::LONG_CONTROL;
                break;
            }
            if(longAndCenterButton.isPressed()) {
                autonSelectionEnum = Selection::LONG_AND_CENTER;
                break;
            }
            if(SAWPButton.isPressed()) {
                autonSelectionEnum = Selection::SOLO_AWP;
                break;
            }
            if(NISAWPButton.isPressed()) {
                autonSelectionEnum = Selection::NI_SAWP;
                break;
            }
            pros::delay(20);
        }
        clearScreen();

        if(allianceColour == AllianceColour::RED) {
            if(startingSide == Selection::LEFT) {
                if(autonSelectionEnum == Selection::LONG_CONTROL) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, LG");
                }
                if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, LG+C");
                }
                if(autonSelectionEnum == Selection::SOLO_AWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Bad!");
                }
                if(autonSelectionEnum == Selection::NI_SAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Bad!");
                }
            }
            if(startingSide == Selection::RIGHT) {
                if(autonSelectionEnum == Selection::LONG_CONTROL) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, LG");
                }
                if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, LG+C");
                }
                if(autonSelectionEnum == Selection::SOLO_AWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, SAWP");
                }
                if(autonSelectionEnum == Selection::NI_SAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, NISAWP");
                }
            }
        }
        if(allianceColour == AllianceColour::BLUE) {
            if(startingSide == Selection::LEFT) {
                if(autonSelectionEnum == Selection::LONG_CONTROL) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, LG");
                }
                if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, LG+C");
                }
                if(autonSelectionEnum == Selection::SOLO_AWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Bad!");
                }
                if(autonSelectionEnum == Selection::NI_SAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Bad!");
                }
            }
            if(startingSide == Selection::RIGHT) {
                if(autonSelectionEnum == Selection::LONG_CONTROL) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, LG");
                }
                if(autonSelectionEnum == Selection::LONG_AND_CENTER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, LG+C");
                }
                if(autonSelectionEnum == Selection::SOLO_AWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, SAWP");
                }
                if(autonSelectionEnum == Selection::NI_SAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, NISAWP");
                }
            }
        }
    }
}

#endif