#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "piston.cpp"

/** Intake control through the shared two-motor intakevator. */
class Intake {
   private:
      int currentState = IntakeState::IDLE;

      void move(int speed) {
         setIntakevatorPto(true);
         intakevator.move(speed);
      }

   public:
      void setState(int intakeState) {
         currentState = intakeState;
      }

      void run() {
         switch(currentState) {
            case IntakeState::IDLE:
               if (intakevatorPto.getState()) intakevator.move(0);
               break;
            case IntakeState::INTAKE:
               move(127);
               break;
            case IntakeState::OUTTAKE:
               move(-127);
               break;
         }
      }

      int getState() const {
         return currentState;
      }
};

inline Intake intake;

#endif
