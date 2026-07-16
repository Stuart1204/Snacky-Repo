#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/optical.h"
#include "other_functions.cpp"

class Intake {
   private:
      int currentState = IntakeState::IDLE;
      int lastState = IntakeState::IDLE;
   public:
      Intake(){} // constructor

   void setState(int intakeState) {
      lastState = currentState;
      currentState = intakeState;
      // switch case if any states need specific actions done when called
      if(lastState != currentState) {
         switch(currentState) {
         case IntakeState::IDLE: 
            break;
         case IntakeState::INTAKE:
            break;
         case IntakeState::OUTTAKE:
            break;
         case IntakeState::INTAKEHIGH:
            break;
      }
      }
   }

   void move(int speed) {
      intakeMotor.move(speed);
   }

   void run() {
      switch(currentState) {
         case IntakeState::IDLE: 
            move(0);
            break;
         case IntakeState::INTAKE:
            if (checkColour(toggleSensor) == allianceColour && toggleSensor.get_proximity() > 80) {
               move(0);
            } else {
               move(127);
            }
            break;
         case IntakeState::OUTTAKE:
            move(-127);
            break;
         case IntakeState::INTAKEHIGH:
            move(115);
            break;
      }
   }

   int getState() {
      return currentState;
   }
};

inline Intake intake;

#endif