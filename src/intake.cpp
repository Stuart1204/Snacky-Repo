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
      int lowerPower = 0;
      int upperPower = 0;
      int rollerPower = 0;

      // for middle goal we need to reverse a little bit before scoring
      int midOuttakeCounter = 0;
      // same with the high goal
      int highOuttakeCounter = 0;
      // colour sort counter
      int colourSortCounter = 0;
      bool currentlyColourSorting = false;
      bool isImmediateOuttake = true;
      bool isImmediateScoreMid = true;
      bool isImmediateIntake = true;
      const int COLOUR_SORT_EJECT_TIME = 10;
      uint32_t immediateOuttakeStartTime = pros::millis();
      uint32_t immediateScoreMidStartTime = pros::millis();
      uint32_t immediateIntakeStartTime = pros::millis();

      int parkStageTracker = 0;

      int intakeWithEjectCounter; // for intakeWithEject state

      bool lowOuttakeJiggleDirection = false;
      int lowOuttakeJiggleCounter = 0;
 
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
            if(middleGoal.getState()) {
               isImmediateIntake = true;
            } else {
               isImmediateIntake = false;
               immediateIntakeStartTime = pros::millis();
            }
            middleGoal.setState(true);
            wing.setState(true);
            break;
         case IntakeState::OUTTAKE:
            break;
         case IntakeState::SCOREHIGH:
         if(checkIsImmediateOuttake()) {
            isImmediateOuttake = true;
         } else {
            isImmediateOuttake = false;
            immediateOuttakeStartTime = pros::millis();
         }
            middleGoal.setState(true);
            wing.setState(false);
            middleDescore.setState(false);
            highOuttakeCounter = 0;
            break;
         case IntakeState::SCOREMID:
            middleGoal.setState(false);
            if(!middleDescore.getState()) {
               isImmediateScoreMid = true;
            } else {
               isImmediateScoreMid = false;
               immediateScoreMidStartTime = pros::millis();
            }
            middleDescore.setState(false);
            break;
         case IntakeState::SCORELOW:
            blockCounter = 0;
            if(autonSelectionEnum == Selection::SKILLS) {
               lowOuttakeJiggleDirection = true;
               lowOuttakeJiggleCounter = 0;
            }
            break;
         case IntakeState::PARK:
            parkStageTracker = 0;
            break;
         case IntakeState::INTAKEWITHEJECT:
            intakeWithEjectCounter = 50;
      }
      }
   }

   void moveLower(int speed) {
      lowerIntake.move(speed);
   }

   void moveUpper(int speed) {
      upperIntake.move(speed);
   }

   void ejectBlock() {
      colourSortCounter++;
      if(colourSortCounter < COLOUR_SORT_EJECT_TIME) {
         moveUpper(50);
      } else {
         colourSortCounter = 0;
         currentlyColourSorting = false;
      }
   }

   void colourSort() {
      if(allianceColour == AllianceColour::RED) {
         if(checkColour(colourSortSensor) == AllianceColour::BLUE) {
            currentlyColourSorting = true;
         }
         if(checkColour(colourSortSensor) == AllianceColour::RED) {
            currentlyColourSorting = false;
            colourSortCounter = 0;
         }
      }
      if(allianceColour == AllianceColour::BLUE) {
         if(checkColour(colourSortSensor) == AllianceColour::RED) {
            currentlyColourSorting = true;
         }
         if(checkColour(colourSortSensor) == AllianceColour::BLUE) {
            currentlyColourSorting = false;
            colourSortCounter = 0;
         }
      }
      if(currentlyColourSorting) {
         ejectBlock();
      }
   }

   void loadPark() {
      if(parkStageTracker == 0) {
         moveLower(-90);
         moveUpper(-127);
         if(parkSensor.get_proximity() > 200) {
            parkStageTracker++;
         }
      }
      if(parkStageTracker == 1) {
         moveLower(-40);
         moveUpper(-127);
         if(parkSensor.get_proximity() < 200) {
            parkStageTracker++;
         }
      }
      if(parkStageTracker == 2) {
         activatePark = true;
         setState(IntakeState::IDLE);
      }
   }

   void run() {
      switch(currentState) {
         case IntakeState::IDLE: 
            moveLower(0);
            moveUpper(0);
            break;

         case IntakeState::INTAKE:
            if(isImmediateIntake) {
               moveLower(127);
               moveUpper(127);
            } else {
               if (pros::millis() - immediateIntakeStartTime > 125) {
                  moveLower(127);
                  moveUpper(127);
               } else {
                  moveLower(0);
                  moveUpper(0);
               }
            }
            break;
         case IntakeState::OUTTAKE:
            moveLower(-127);
            moveUpper(-127);
            break;
         case IntakeState::SCOREHIGH:
            if(isImmediateOuttake) {
               moveLower(127);
               moveUpper(127);
            }
            else {
               if (pros::millis() - immediateOuttakeStartTime > 125) {
                  moveLower(127);
                  moveUpper(127);
               } else {
                  moveLower(0);
                  moveUpper(0);
               }
            }
            break;
         case IntakeState::SCOREMID:
            if(autonSelectionEnum == Selection::SKILLS) {
               moveLower(127);
               moveUpper(60);
            } else {
               if(pros::competition::is_autonomous()) {
                  moveLower(127);
                  moveUpper(80);
               } else {
                  if (pros::millis() - immediateScoreMidStartTime > 225) {
                     moveLower(127);
                     moveUpper(100);
                  } else {
                     moveLower(0);
                     moveUpper(0);
                  }
               }
            }
            if(slowMidGoal && autonSelectionEnum == Selection::SKILLS) {
               moveUpper(60);
            } 
            break;
         case IntakeState::SCORELOW: // meaning the robot is tryna score 7 on low
            if(autonSelectionEnum == Selection::SKILLS) {
               // moveLower(-50);
               // moveUpper(-127);

               if(lowOuttakeJiggleDirection) {
                  moveLower(-55);
                  moveUpper(-100);
                  if(lowOuttakeJiggleCounter > 25) {
                     lowOuttakeJiggleCounter = 0;
                     lowOuttakeJiggleDirection = false;
                  }
               } else {
                  moveLower(-55);
                  moveUpper(80);
                  if(lowOuttakeJiggleCounter > 3) {
                     lowOuttakeJiggleCounter = 0;
                     lowOuttakeJiggleDirection = true;
                  }
               }
               lowOuttakeJiggleCounter++;
            } else {
               moveLower(-85);
               moveUpper(-127);
            }
            if(slowLowGoal && autonSelectionEnum == Selection::SKILLS) {
               moveLower(-45);
            } 
            break;
         case IntakeState::PARK:
            loadPark();
            break;
         case IntakeState::LOWINTAKE:
            moveLower(127);
            moveUpper(10);
            break;
         case IntakeState::LOWOUTTAKE:
            moveLower(-95);
            moveUpper(10);
            break;
         case IntakeState::UPPEROUTTAKE:
            moveUpper(-100);
            moveLower(-20);
            break;
         case IntakeState::INTAKEWITHEJECT:
            moveLower(127);
            moveUpper(127);
            if(allianceColour == AllianceColour::BLUE) {
               if(checkColour(parkSensor) == AllianceColour::RED) {
                  moveLower(-127);
                  intakeWithEjectCounter = 0;
               }
            }
            if(allianceColour == AllianceColour::RED) {
               if(checkColour(parkSensor) == AllianceColour::BLUE) {
                  moveLower(-127);
                  intakeWithEjectCounter = 0;
               }
            }
            if(intakeWithEjectCounter < 10) {
               moveLower(-127);
            }
            intakeWithEjectCounter++;
            break;
         case IntakeState::SCOREHIGHSLOW:
            moveLower(127);
            moveUpper(80);
            break;
         case IntakeState::SLOWOUTTAKE:
            moveLower(-40);
            moveUpper(-127);
            break;
      }
   }

   int getState() {
      return currentState;
   }
};

inline Intake intake;

#endif