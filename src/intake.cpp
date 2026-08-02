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
      bool toggleFlipColourSeen = false;
      uint32_t toggleStartTime = pros::millis();
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
            toggleFlipColourSeen = false;
            break;
         case IntakeState::OUTTAKE:
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
            move(127);
            break;
         case IntakeState::OUTTAKE:
            move(-127);
            break;
      }
   }

   int getState() {
      return currentState;
   }
};

inline Intake intake;

#endif