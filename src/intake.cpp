#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/optical.h"

class Intake {
 
   public:
      Intake(){} // constructor
      int counter = 0;
      bool sensorBlocked = false;
      int lowerPower = 0;
      int upperPower = 0;
      int rollerPower = 0;

   void moveLower(int speed) {
      lowerIntake.move(speed);
   }

   void moveUpper(int speed) {
      upperIntake.move(speed);
   }
   void moveScore(int speed) {
      scoringRoller.move(speed);
   }

   void upperMoveVelocity(int speed) {
      upperIntake.move_velocity(speed);
   }
};

inline Intake intake;

#endif