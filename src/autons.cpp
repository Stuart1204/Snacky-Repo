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