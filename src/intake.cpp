#ifndef INTAKE_CPP
#define INTAKE_CPP

#include "globals.hpp"
#include "main.h"
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

      int scoreStallCounter = 0;
      int upperStallCounter = 0;

      // for middle goal we need to reverse a little bit before scoring
      int midOuttakeCounter = 0;
      // same with the high goal
      int highOuttakeCounter = 0;
      // colour sort counter
      int colourSortCounter = 0;
      bool currentlyColourSorting = false;
      const int COLOUR_SORT_EJECT_TIME = 10;

      int parkStageTracker = 0;

      int intakeWithEjectCounter; // for intakeWithEject state
 
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
            scoreStallCounter = 0;
            upperStallCounter = 0;
            setWings(true);
            break;
         case IntakeState::OUTTAKE:
            break;
         case IntakeState::SCOREHIGH:
            setWings(false);
            highOuttakeCounter = 0;
            colourSortCounter = 0;
            break;
         case IntakeState::SCOREMID:
            midOuttakeCounter = 0;
            break;
         case IntakeState::SCORELOW:
            blockCounter = 0;

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
         moveLower(-80);
         moveUpper(-127);
         if(parkSensor.get_proximity() > 200) {
            parkStageTracker++;
         }
      }
      if(parkStageTracker == 1) {
         moveLower(-35);
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
            moveLower(127);
            moveUpper(127);
            break;
         case IntakeState::OUTTAKE:
            moveLower(-127);
            moveUpper(-127);
            break;
         case IntakeState::SCOREHIGH:
            highOuttakeCounter++;
            if(highOuttakeCounter < 7) {
               moveLower(-60);
               moveUpper(-100);
            } else {
               moveLower(127);
               moveUpper(127);
               if(enableColourSort) {
                  colourSort();
                  colourSortSensor.set_led_pwm(100);
               } else {
                  colourSortSensor.set_led_pwm(0);
               }
            }
            break;
         case IntakeState::SCOREMID:
            
            break;
         case IntakeState::SCORELOW: // meaning the robot is tryna score 7 on low
            if(autonSelectionEnum == Selection::SKILLS) {
               moveLower(-60);
               moveUpper(-90);
            } else {
               moveLower(-100);
               moveUpper(-127);
            }
            countBlocks(parkSensor);
            // if((blockCounter >= 6 || slowLowGoal) && autonSelectionEnum == Selection::SKILLS) {
            //    moveLower(-45);
            // }
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
            moveUpper(120);
            break;
      }
   }

   int getState() {
      return currentState;
   }
};

inline Intake intake;

#endif