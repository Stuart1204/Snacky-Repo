#ifndef PISTON_CPP
#define PISTON_CPP

#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"

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
inline Piston intakeLift(intakeLiftPiston);
inline Piston clamp(clampPiston);
inline Piston hood(hoodPiston);
inline Piston doinker(doinkerPiston);
// Wrist hardware is digital only: false = retracted, true = extended.
inline Piston wristPiston(wristPistonOutput);
// Selects whether intakevator powers the elevator (false) or intake (true).
inline Piston intakevatorPto(intakevatorPtoOutput);

inline void setIntakevatorPto(bool intakeSelected) {
    if (intakevatorPto.getState() == intakeSelected) return;
    intakevator.move(0);
    intakevatorPto.setState(intakeSelected);
    pros::delay(100);
}

#endif