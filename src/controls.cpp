#ifndef controls_cpp
#define controls_cpp

#include "globals.hpp"
#include "localization.hpp"
#include "intakevator.cpp"
#include "piston.cpp"
#include "pros/colors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cstring>


#include <algorithm>
#include <cmath>

constexpr double MAX_ELEVATOR_HEIGHT = 50.0;
constexpr double MIN_THROTTLE_CAP = 70.0;   // Maximum throttle when fully extended
constexpr double DECEL_PER_SECOND = 600.0;  // Deceleration after stick release
constexpr double LOW_SPEED_THRESHOLD = 50.0;
constexpr double SMALL_CHANGE = 3.0;
constexpr double DT = 0.02;

inline double approach(double target, double current, double maxChange) {
    if (target > current) {
        return std::min(target, current + maxChange);
    }
    return std::max(target, current - maxChange);
}

inline double shapeDriveThrottle(double requested, double elevatorHeight) {
    static double current = 0.0;

    // 0 = retracted, 1 = fully extended.
    const double heightAlpha = std::clamp(
        elevatorHeight / MAX_ELEVATOR_HEIGHT, 0.0, 1.0);

    // Only large outputs are limited; low-speed stick control is unchanged.
    const double throttleCap = 127.0 -
        heightAlpha * (127.0 - MIN_THROTTLE_CAP);

    requested = std::clamp(requested, -throttleCap, throttleCap);

    const bool slowingDown =
        (current > 0.0 && requested < current) ||
        (current < 0.0 && requested > current);

    if (std::abs(current) < LOW_SPEED_THRESHOLD ||
        std::abs(requested - current) < SMALL_CHANGE) {
        // Keep fine low-speed adjustments immediate.
        current = requested;
    } else if (slowingDown) {
        // Smooth braking and stick release, but allow direct acceleration.
        current = approach(
            requested, current, DECEL_PER_SECOND * DT);
    } else {
        current = requested;
    }

    return current;
}



inline void intakeControls() {
    const IntakevatorState state = intakevator.getState();

    if (controller.get_digital(Buttons::L2)) {
        intakevator.setState(IntakevatorState::OUTTAKE);
    } else if (controller.get_digital(Buttons::L1)) {
        if (state != IntakevatorState::RESET && state != IntakevatorState::STOW &&
            state != IntakevatorState::MANUAL &&
            state != IntakevatorState::PIN_HEIGHT) {
        }
    } else if (state == IntakevatorState::INTAKE || state == IntakevatorState::OUTTAKE) {
        intakevator.setState(IntakevatorState::IDLE);
    }
}
inline bool manualActive = false; // tracks if any manual button is held this cycle

inline void intakevatorControls()
{
    const IntakevatorState state = intakevator.getState();
    const bool wasManual = manualActive;
    manualActive = false;

    if (controller.get_digital_new_press(Buttons::R2)) {
        if (state == IntakevatorState::PIN_HEIGHT || state == IntakevatorState::STOW ||
            state == IntakevatorState::MANUAL) {
            intakevator.setState(IntakevatorState::SCORE);
        }
    }
    if (controller.get_digital_new_press(Buttons::B)) {
        intakevator.setState(IntakevatorState::RESET);
    }

    if (controller.get_digital(Buttons::RIGHT)) {
        manualActive = true;
        intakevator.manualMove(-1);
        intakevator.setMaxSpeed(70);
    }
    if (controller.get_digital(Buttons::Y)) {
        manualActive = true;
        intakevator.manualMove(1);
        intakevator.setMaxSpeed(127);
    }
    if (wasManual && !manualActive) {
        intakevator.manualStop();
    }

    if (controller.get_digital_new_press(Buttons::R1)) {
        intakevator.setMaxSpeed(127);
    }

    if (state == IntakevatorState::ESTOP) {
        controller.rumble("-");
        intakevator.setState(IntakevatorState::IDLE);
    }
    if (controller.get_digital_new_press(Buttons::UP)) {
        isNeutralGoal = !isNeutralGoal;
        controller.rumble(".");
        controller.clear_line(0);
        pros::delay(50);
        controller.print(0, 0, "%s", isNeutralGoal ? "NEUTRAL " : "ALLIANCE");
    }
}

inline void activatePistons()
{
    // The doinker is allowed only with the wrist retracted; extending the wrist retracts it.
    if (controller.get_digital_new_press(Buttons::A) &&
        wristPiston.getState() == IntakevatorConfig::WRIST_RETRACTED)
    {
        doinker.toggle();
    }
    if (controller.get_digital_new_press(Buttons::LEFT))
    {
        clamp.toggle();
    }
    if (wristPiston.getState() == IntakevatorConfig::WRIST_EXTENDED) {
        doinker.setState(false);
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
    int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    if (std::abs(leftY) < 12) leftY = 0;
    if (std::abs(rightX) < 12) rightX = 0;

    const double throttle = shapeDriveThrottle(
        leftY, intakevator.getPosition());

    chassis.arcade(throttle, rightX, true, 0.5);
}

inline void subsystemControls()
{
    // setToggle();
    if (!toggleFlipActive) {
        intakeControls();
    }
    intakevatorControls();
    activatePistons();
}

/** Displays raw wheel/IMU odometry and the MCL estimate in LemLib units. */
inline void displayData()
{
    const lemlib::Pose trackingPose = localization.getTrackingPose();
    const lemlib::Pose correctedPose = localization.getCorrectedPose();

    pros::screen::print(TEXT_MEDIUM_CENTER, 0, "TRACKING WHEEL ODOM");
    pros::screen::print(TEXT_MEDIUM_CENTER, 1, "X: %.2f in", trackingPose.x);
    pros::screen::print(TEXT_MEDIUM_CENTER, 2, "Y: %.2f in", trackingPose.y);
    pros::screen::print(TEXT_MEDIUM_CENTER, 3, "Theta: %.2f deg", trackingPose.theta);

    pros::screen::print(TEXT_MEDIUM_CENTER, 4, "MCL CORRECTED (%s)", enableMclCorrections ? "ON" : "OFF");
    pros::screen::print(TEXT_MEDIUM_CENTER, 5, "X: %.2f in", correctedPose.x);
    pros::screen::print(TEXT_MEDIUM_CENTER, 6, "Y: %.2f in", correctedPose.y);
    pros::screen::print(TEXT_MEDIUM_CENTER, 7, "Theta: %.2f deg", correctedPose.theta);
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