#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"

class Elevator {
   private:
    lemlib::PID pid = lemlib::PID(1, 0, 0);
      int currentState = ElevatorState::STOW;
      int lastState = ElevatorState::STOW;
   public:
      Elevator(){} // constructor

   void setState(int elevatorState) {
      lastState = currentState;
      currentState = elevatorState;
      // switch case if any states need specific actions done when called
      if(lastState != currentState) {
         switch(currentState) {
         case ElevatorState::STOW: 
            break;
         case ElevatorState::MIDDLE:
            break;
         case ElevatorState::BOTTOM:
            break;
      }
      }
   }

   void moveLeft(int speed) {
      leftElevator.move(speed);
   }

   void moveRight(int speed) {
      rightElevator.move(speed);
   }

   void run() {
      switch(currentState) {
         case ElevatorState::STOW: 
            
            break;

         case ElevatorState::MIDDLE:

            break;
         case ElevatorState::BOTTOM:

            break;
      }
   }

   int getState() {
      return currentState;
   }
};

inline Elevator elevator;

#endif