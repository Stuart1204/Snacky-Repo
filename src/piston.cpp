#ifndef PISTON_CPP
#define PISTON_CPP

#include "globals.hpp"
#include "pros/adi.hpp"

class Piston {
    private:
        bool state = false;
        pros::adi::DigitalOut piston;
    public:
        Piston(pros::adi::DigitalOut parameterPiston) : piston(parameterPiston) {}

        void toggle() {
            state = !state;
            if (state) {
                piston.set_value(true);
            } else {
                piston.set_value(false);
            }
        }
        void setState(bool wantedState) {
            state = wantedState;
            piston.set_value(state);
        }
        bool getState() {
            return state;
        }
};

// pistons
inline Piston leftWing(leftWingPiston);
inline Piston rightWing(rightWingPiston);
inline Piston tongue(tonguePiston);
inline Piston park(parkPiston);
inline Piston intakeLift(intakeLiftPiston);
inline Piston odomLift(odomLiftPiston);
inline Piston middleGoalDescore(middleGoalDescorePiston);

#endif