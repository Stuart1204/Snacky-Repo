#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "pros/adi.hpp"
#include "pros/optical.h"

class Intake {
   private:
      int currentState = IntakeState::IDLE;
      int lastState = IntakeState::IDLE;
      int lowerPower = 0;
      int upperPower = 0;
      int rollerPower = 0;

      int scoreStallCounter = 0;
      int upperStallCounter = 0;
 
   public:
      Intake(){} // constructor

   void setState(int intakeState) {
      lastState = currentState;
      currentState = intakeState;

      // resetting jam counters if the robot starts intaking
      if(currentState == IntakeState::INTAKE && lastState != currentState) {
         scoreStallCounter = 0;
         upperStallCounter = 0;
      }
   }

   void moveLower(int speed) {
      lowerIntake.move(speed);
   }

   void moveUpper(int speed) {
      upperIntake.move(speed);
   }
   void moveScore(int speed) {
      scoringRoller.move(speed);
   }

   void moveWithStallDetection() {
      // Always move lower intake (no stall detection needed)
      moveLower(127);

      int UPPER_STALL_SPEED_THRESHOLD = 50;
      double SCORING_STALL_CURRENT_THRESHOLD = 600;
      int STALL_TIME__THRESHOLD = 10;
      
      // Check if upper motor is stalled
      if (abs(upperIntake.get_actual_velocity()) < UPPER_STALL_SPEED_THRESHOLD) {
         upperStallCounter++;
         if (upperStallCounter > STALL_TIME__THRESHOLD) {
            moveUpper(0); // Stop upper motor if stalled
         } else {
            moveUpper(127);
         }
      } else {
         moveUpper(127);
      }
      
      // Check if scoring roller is stalled
      if (abs(scoringRoller.get_current_draw()) > SCORING_STALL_CURRENT_THRESHOLD) {
         scoreStallCounter++;
         if (scoreStallCounter > STALL_TIME__THRESHOLD) {
            moveScore(0); // Stop scoring motor if stalled
         } else {
            moveScore(127);
         }
      } else {
         moveScore(127);
      }
   }


   void loadPark() {
      while (parkSensor.get_proximity() < 200) {
         moveLower(-80);
         moveUpper(-127);
         pros::delay(20);
         if(currentState != IntakeState::PARK && currentState != IntakeState::IDLE) {
            return;
         }
      }
      while (parkSensor.get_proximity() > 200) {
         moveLower(-35);
         moveUpper(-127);
         pros::delay(20);
         if(currentState != IntakeState::PARK && currentState != IntakeState::IDLE) {
            return;
         }
      }
      // activating park piston
      activatePark = true;
   }

   void run() {
      switch(currentState) {
         case IntakeState::IDLE: 
            moveLower(0);
            moveUpper(0);
            moveScore(0);
            break;

         case IntakeState::INTAKE:
            moveWithStallDetection();
            break;
         case IntakeState::OUTTAKE:
            moveLower(-127);
            moveUpper(-127);
            moveScore(-127);
            break;
         case IntakeState::SCOREHIGH:
            moveLower(127);
            moveUpper(127);
            moveScore(127);
            break;
         case IntakeState::SCOREMID:
            moveLower(127);
            moveUpper(127);
            moveScore(-127);
            break;
         case IntakeState::SCORELOW:
            moveLower(-60);
            moveUpper(-127);
            moveScore(-127);
            break;
         case IntakeState::PARK:
            loadPark();
            currentState = IntakeState::IDLE;
            break;
      }
   }

   int returnState() {
      return currentState;
   }
};

inline Intake intake;

#endif