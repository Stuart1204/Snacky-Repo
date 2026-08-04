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

inline bool targetMode = false;

inline void intakeControls() {
    if (controller.get_digital(Buttons::L2))
    {
        intake.setState(IntakeState::OUTTAKE);
    }
    else if (controller.get_digital(Buttons::L1))
    {
        intake.setState(IntakeState::INTAKE);
        if(elevator.getState() != ElevatorState::RESET && elevator.getState() != ElevatorState::STOW
             && elevator.getState() != ElevatorState::MANUAL && elevator.getState() != ElevatorState::PICKUP
                && elevator.getState() != ElevatorState::PIN_HEIGHT) {
                
            if(matchloadMode) {
                elevator.setState(ElevatorState::MATCHLOAD);
            } else {
                elevator.setState(ElevatorState::LOAD);
            }
        }
    }
    else if (elevator.getState() != ElevatorState::RESET)
    {
        intake.setState(IntakeState::IDLE);
    }                                           
}                                                                                                                                                                                             
inline bool manualActive = false; // tracks if any manual button is held this cycle

inline int levelCounter = 1;
inline void elevatorControls()
{
    int state = elevator.getState();
    bool wasManual = manualActive;
    manualActive = false;

    if (controller.get_digital_new_press(Buttons::R2))
    {
        elevator.setState(ElevatorState::SCORE);
    }
    if (controller.get_digital_new_press(Buttons::B)) {
        elevator.setState(ElevatorState::RESET);
    }

    if (controller.get_digital(Buttons::RIGHT))
    {
        targetMode = false;
        manualActive = true;
        elevator.manualElevator(-1);
        elevator.setMaxSpeed(70);
    }
    if (controller.get_digital(Buttons::Y))
    {
        targetMode = false;
        manualActive = true;
        elevator.manualElevator(1);
        elevator.setMaxSpeed(127);
    }

    // ─────────────────────────────────────────────────────
    // When all manual buttons released, snap to current position
    // ─────────────────────────────────────────────────────
    if (wasManual && !manualActive)
    {
        elevator.manualStop();
    }

    // ─────────────────────────────────────────────────────
    // Y PRESS: Toggle matchload mode
    // ─────────────────────────────────────────────────────
    if (controller.get_digital_new_press(Buttons::X))
    {
        matchloadMode = !matchloadMode;
        intakeLift.setState(matchloadMode);

        if (state == ElevatorState::LOAD || state == ElevatorState::MATCHLOAD)
        {
            if (matchloadMode)
                elevator.setState(ElevatorState::MATCHLOAD);
            else
                elevator.setState(ElevatorState::LOAD);
        }
    }

    // ─────────────────────────────────────────────────────
    // B PRESS: PICKUP or cycle pin heights
    // ─────────────────────────────────────────────────────
    if(state == ElevatorState::PIN_HEIGHT) {
        if(levelCounter != 1) {
                elevator.goToPinHeight(levelCounter);
                levelCounter = 1;
            }
    }
    if (controller.get_digital_new_press(Buttons::R1))
    {
        elevator.setMaxSpeed(127);
        if (state == ElevatorState::LOAD || state == ElevatorState::MATCHLOAD)
        {
            targetMode = true;
            elevator.setState(ElevatorState::PICKUP);
            levelCounter = 1;
        }
        if(state == ElevatorState::PICKUP) {
            levelCounter++;
        }
        else if (state == ElevatorState::PIN_HEIGHT)
        {
            if (targetMode)
            {
                int nextLevel = elevator.getPinLevel() + 1;
                if (nextLevel <= ElevatorConfig::NUM_PIN_LEVELS)
                {
                    elevator.goToPinHeight(nextLevel);
                }
            }
            else
            {
                targetMode = true;
                elevator.goToNextPinAboveCurrent();
            }
        }
    }

    // ─────────────────────────────────────────────────────
    // ESTOP RECOVERY
    // ─────────────────────────────────────────────────────
    if (state == ElevatorState::ESTOP)
    {
        controller.rumble("-");
        elevator.setState(ElevatorState::REST);
    }
    if (controller.get_digital_new_press(Buttons::UP)) {
        isNeutralGoal = !isNeutralGoal;
        controller.rumble(".");
        controller.clear_line(0);
        pros::delay(50);
        controller.print(0, 0, "%s", isNeutralGoal ? "NEUTRAL " : "ALLIANCE");
        if (elevator.getState() == ElevatorState::PIN_HEIGHT && elevator.getPinLevel() > 0) {
            elevator.goToPinHeight(elevator.getPinLevel());
        }
    }
}

inline void activatePistons()
{
    if (controller.get_digital_new_press(Buttons::R1))
    {
        if(!intakeLift.getState()) {
            elevator.setState(ElevatorState::MATCHLOAD);
        } else {
            elevator.setState(ElevatorState::LOAD);
        }
    }
    if (controller.get_digital_new_press(Buttons::A))
    {
        doinker.toggle();
    }
    if (controller.get_digital_new_press(Buttons::LEFT))
    {
        clamp.toggle();
    }
    // if (controller.get_digital_new_press(Buttons::DOWN))
    // {
    //     hood.toggle();
    // }
}

inline bool toggleFlipActive = false;
inline bool toggleFlipColourSeen = false;


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
    pros::screen::print(TEXT_MEDIUM_CENTER, 1, "X: %f", pose.x);
    pros::screen::print(TEXT_MEDIUM_CENTER, 2, "Y: %f", pose.y);
    pros::screen::print(TEXT_MEDIUM_CENTER, 3, "Theta: %f", pose.theta);
    pros::screen::print(TEXT_LARGE_CENTER, 5, "velo: %f", intakeMotor.get_actual_velocity());
    //pros::screen::print(TEXT_LARGE_CENTER, 7, "Colour: %i", checkColour(toggleSensor));
    // pros::screen::print(TEXT_LARGE_CENTER, 9, "dist: %f", rightCurrentDistance);

    pros::screen::print(TEXT_LARGE_CENTER, 7, "elevatorPos: %f", elevator.getElevatorPosition());
    // pros::screen::print(TEXT_LARGE_CENTER, 3, "wristPos: %f", elevator.getWristPosition());
    // pros::screen::print(TEXT_LARGE_CENTER, 9, "Proximity: %iu", vertical_rotation.get_position());
    // // pros::screen::print(TEXT_LARGE_CENTER, 7, "ele: %f", (leftElevator.get_position() - rightElevator.get_position()) / 2.0);
    pros::screen::print(TEXT_LARGE_CENTER, 9, "leftPower: %i", elevator.getLeftPower());
    pros::screen::print(TEXT_LARGE_CENTER, 11, "rightPower: %i", elevator.getRightPower());


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

inline void clampSelection() {
    button clampButton(20, 15, 225, 220, pros::Color::fuchsia, "Clamp");
    //button MatchButton(250, 15, 455, 220, pros::Color::alice_blue, "Match");
    // selecting between skills and match
    while(true) {
        if(clampButton.isPressed()) {
            //autonSelectionEnum = Selection::SKILLS;
            clamp.toggle();
            break;
        }
        // if(matchButton.isPressed()) {
        //     autonSelectionEnum = Selection::MATCH;
        //     break;
        // }
        pros::delay(20);
    }
}

inline void autonSelection() {
    button skillsButton(20, 15, 225, 220, pros::Color::fuchsia, "Skills");
    button matchButton(250, 15, 455, 220, pros::Color::alice_blue, "Match");
    //selecting between skills and match
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

        button TAWPButton(0, 15, 120, 180, pros::Color::sky_blue, "TAWP");
        button ElimsButton(120, 15, 240, 180, pros::Color::sky_blue, "Elims");
        button CornerButton(240, 15, 360, 180, pros::Color::sky_blue, "Corner");
        button NothingButton(360, 15, 480, 180, pros::Color::sky_blue, "nottin");

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
            if(TAWPButton.isPressed()) {
                autonSelectionEnum = Selection::TAWP;
                break;
            }
            if(ElimsButton.isPressed()) {
                autonSelectionEnum = Selection::ELIMS;
                break;
            }
            if(CornerButton.isPressed()) {
                autonSelectionEnum = Selection::CORNER;
                break;
            }
            if(NothingButton.isPressed()) {
                autonSelectionEnum = Selection::NOTHING;
                break;
            }
            pros::delay(20);
        }
        clearScreen();

        if(allianceColour == AllianceColour::RED) {
            if(startingSide == Selection::LEFT) {
                if(autonSelectionEnum == Selection::TAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, TAWP");
                }
                if(autonSelectionEnum == Selection::ELIMS) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, Elims");
                }
                if(autonSelectionEnum == Selection::CORNER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Left, corner");
                }
                if(autonSelectionEnum == Selection::NOTHING) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "nottin");
                }
            }
            if(startingSide == Selection::RIGHT) {
                if(autonSelectionEnum == Selection::TAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, TAWP");
                }
                if(autonSelectionEnum == Selection::ELIMS) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, ELIMS");
                }
                if(autonSelectionEnum == Selection::CORNER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Red, Right, corner");
                }
                if(autonSelectionEnum == Selection::NOTHING) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "nothing");
                }
            }
        }
        if(allianceColour == AllianceColour::BLUE) {
            if(startingSide == Selection::LEFT) {
                if(autonSelectionEnum == Selection::TAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, TAWP");
                }
                if(autonSelectionEnum == Selection::ELIMS) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Left, ELIMS");
                }
                if(autonSelectionEnum == Selection::CORNER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, left, corner");
                }
                if(autonSelectionEnum == Selection::NOTHING) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "nothing");
                }
            }
            if(startingSide == Selection::RIGHT) {
                if(autonSelectionEnum == Selection::TAWP) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, TAWP");
                }
                if(autonSelectionEnum == Selection::ELIMS) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, ELIMS");
                }
                if(autonSelectionEnum == Selection::CORNER) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "Blue, Right, corner");
                }
                if(autonSelectionEnum == Selection::NOTHING) {
                    button currentSelection(20, 15, 455, 180, pros::Color::fuchsia, "nothing");
                }
            }
        }
    }
}

#endif