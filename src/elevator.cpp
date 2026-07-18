#ifndef ELEVATOR_CPP
#define ELEVATOR_CPP

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"
namespace ElevatorConfig
{
   // Encoder ticks (motor degrees) per inch of elevator travel.
   //
   constexpr double TICKS_PER_INCH = 57.5;

   // Wrist rotation sensor: degrees reported per degree of actual wrist rotation.
   constexpr double SENSOR_DEG_PER_WRIST_DEG = 1.0;

   // Elevator soft limits (inches)
   constexpr double ELEVATOR_MIN = 0.0;
   constexpr double ELEVATOR_MAX = 40.0;

   // Wrist soft limits (degrees from home)
   constexpr double WRIST_MIN = -90.0;
   constexpr double WRIST_MAX = 90.0;

   // ── Macro target positions ──────────────────────────────
   // { elevatorInches, wristDegrees }
   constexpr double STOW_ELEV = 15.0, STOW_WRIST = -5.0;
   constexpr double SCORE_ELEV = -10, SCORE_WRIST = -70.0;
   constexpr double RESET_ELEV = 7, RESET_WRIST = -80.0;
   constexpr double LOAD_ELEV = 1.4, LOAD_WRIST = -80.0;
   constexpr double MATCHLOAD_ELEV = 5, MATCHLOAD_WRIST = -140.0;
   constexpr double PICKUP_ELEV = 10, PICKUP_WRIST = -189.0;
   constexpr double MANUAL_ELEV = 10, MANUAL_WRIST = -5.0;

   // PID gains  { kP, kI, kD, windupRange }
   constexpr double ELEV_KP = 45, ELEV_KI = 0.0, ELEV_KD = 0;
   constexpr double WRIST_KP = 4.5, WRIST_KI = 0.0, WRIST_KD = 1.7;

   // Settle threshold
   constexpr double ELEV_SETTLE_IN = 2.5;  // inches
   constexpr double WRIST_SETTLE_DEG = 20; // degrees
}

class Elevator
{
private:
   lemlib::PID elevatorPID{
       ElevatorConfig::ELEV_KP,
       ElevatorConfig::ELEV_KI,
       ElevatorConfig::ELEV_KD, 0};
   lemlib::PID wristPID{
       ElevatorConfig::WRIST_KP,
       ElevatorConfig::WRIST_KI,
       ElevatorConfig::WRIST_KD, 0};
   int currentState = ElevatorState::REST;
   int lastState = ElevatorState::REST;
   int pickUpPhase = 0;
   double elevatorTarget = 0.0; // inches
   double wristTarget = 0.0;    // degrees
   bool reachedSpeed = false;
   int reachedSpeedThreshold = 200;
   uint32_t startTime = 0;

   int leftPower = 0;
   int rightPower = 0;

   static double clampD(double v, double lo, double hi)
   {
      if (v < lo)
         return lo;
      if (v > hi)
         return hi;
      return v;
   }
   static int clamp127(double v) {
      if (v >  127) return  127;
      if (v < -127) return -127;
      return static_cast<int>(v);
   }

   // Apply a macro target pair and transition state
   void applyMacro(int newState, double elevIn, double wristDeg)
   {
      lastState = currentState;
      currentState = newState;
      elevatorTarget = clampD(elevIn, ElevatorConfig::ELEVATOR_MIN,
                              ElevatorConfig::ELEVATOR_MAX);
      wristTarget = clampD(wristDeg, ElevatorConfig::WRIST_MIN,
                           ElevatorConfig::WRIST_MAX);
      elevatorPID.reset();
      wristPID.reset();
   }

public:
   Elevator()
   {
      leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   }

   void setState(int state)
   {
      if (state == currentState)
         return;

      lastState = currentState;
      currentState = state;

      leftElevator.move(0);
      rightElevator.move(0);

      switch (currentState)
      {
      case ElevatorState::STOW:
         setWristTarget(-180);
         setElevatorTarget(ElevatorConfig::STOW_ELEV);
         break;
      case ElevatorState::SCORE:
         setWristTarget(ElevatorConfig::SCORE_WRIST);
         elevatorTarget = getElevatorPosition();
         reachedSpeed = false;
         startTime = pros::millis();
         break;
      case ElevatorState::REST:
         // setElevatorTarget(getElevatorPosition());
         // setWristTarget(getWristPosition());
         break;
      case ElevatorState::RESET:
         setWristTarget(ElevatorConfig::RESET_WRIST);
         setElevatorTarget(getElevatorPosition());
         reachedSpeed = false;
         startTime = pros::millis();
         break;
      case ElevatorState::LOAD:
         setWristTarget(ElevatorConfig::LOAD_WRIST);
         setElevatorTarget(ElevatorConfig::LOAD_ELEV);
         reachedSpeed = false;
         break;
      case ElevatorState::PICKUP:
         setWristTarget(getWristPosition());
         setElevatorTarget(ElevatorConfig::PICKUP_ELEV);
         clamp.setState(false);
         pickUpPhase = 0;
         reachedSpeed = false;
         break;
      case ElevatorState::MANUAL:
         setWristTarget(ElevatorConfig::MANUAL_WRIST);
      case ElevatorState::MATCHLOAD:
         setWristTarget(ElevatorConfig::MATCHLOAD_WRIST);
         setElevatorTarget(ElevatorConfig::MATCHLOAD_ELEV);
         break;
      }
   }
   int getState() const { return currentState; }

   // ══════════════════════════════════════════════════════
   //  MANUAL INCREMENT CONTROLS
   //  Call these from opcontrol button presses.
   //  They work in MANUAL state; in macro states they also
   //  nudge the target (useful for fine-tuning after a macro).
   // ══════════════════════════════════════════════════════

   void changeElevatorTarget(double delta)
   {
      elevatorTarget = elevatorTarget + delta;
   }

   double getElevatorTarget()
   {
      return elevatorTarget;
   }
   double getWristTarget()
   {
      return wristTarget;
   }
   double getElevatorPosition()
   {
      double elevTicks = (leftElevator.get_position() - rightElevator.get_position()) / 2.0;
      return elevTicks / ElevatorConfig::TICKS_PER_INCH;
   }

   double getWristPosition()
   {
      double pos = wristRotation.get_angle() / 100.0;
      if(pos > 45) {
         pos -= 360;
      }
      return pos;
   }

   void changeWristTarget(double delta)
   {
      wristTarget = wristTarget + delta;
   }

   // Direct target setters — use in autonomous
   void setElevatorTarget(double inches)
   {
      elevatorTarget = inches;
   }

   void setWristTarget(double degrees)
   {
      if(degrees > 45) {
         wristTarget = degrees - 360;
      } else {
         wristTarget = degrees;
      }
   }

   // Freeze targets at current sensor readings
   void holdCurrent()
   {
      elevatorTarget = getElevatorPosition();
      wristTarget = getWristPosition();
      elevatorPID.reset();
      wristPID.reset();
   }

   // Tare everything back to zero (use when physically at home)
   void elevatorReset()
   {
      leftElevator.tare_position();
      rightElevator.tare_position();
      elevatorPID.reset();
      wristPID.reset();
   }

   // ── Settle check (useful in auton waits) ───────────────
   bool isElevatorSettled()
   {
      return std::abs(getElevatorPosition() - elevatorTarget) < ElevatorConfig::ELEV_SETTLE_IN;
   }

   bool isWristSettled()
   {
      return std::abs(getWristPosition() - wristTarget) < ElevatorConfig::WRIST_SETTLE_DEG;
   }

   bool isSettled()
   {
      return isElevatorSettled() && isWristSettled();
   }

   bool isElevatorStalled() {
      // if(reachedSpeed == false && (pros::millis() - startTime > 1000 || fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 > reachedSpeedThreshold)) {
      //    reachedSpeed = true;
      // }
      // if(reachedSpeed && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 < reachedSpeedThreshold) {
      //    reachedSpeed = false;
      //    return true;
      // }
      // return false;
      double avgVelocity = (fabs(leftElevator.get_actual_velocity()) + fabs(rightElevator.get_actual_velocity())) / 2;
      double avgCurrent = (fabs(leftElevator.get_current_draw())+fabs(rightElevator.get_current_draw())) / 2;
      if (pros::millis() - startTime < 200) {
         return false;
      }
      return ((avgVelocity < 10 && avgCurrent > 1800) || pros::millis() - startTime > 1000);
   }

   bool isStalled() {
      static uint32_t noProgressStart = 0;
      static double lastPos = 0;
      double currentPos = getElevatorPosition();
      bool isMoving = fabs(currentPos - lastPos) > 0.2;
      if (isMoving) {
         noProgressStart = pros::millis();
         lastPos = currentPos;
         return false;
      }
      return (pros::millis()-noProgressStart)>2000;
   }

   // Blocking wait — call in autonomous tasks only
   void waitUntilSettled(uint32_t timeoutMs = 2000)
   {
      uint32_t start = pros::millis();
      while (!isSettled() && (pros::millis() - start) < timeoutMs)
      {
         pros::delay(10);
      }
   }

   int getLeftPower() {
      return leftPower;
   }

   int getRightPower() {
      return rightPower;
   }

   // ══════════════════════════════════════════════════════
   //  RUN  — called every ~10 ms from a dedicated thread
   // ══════════════════════════════════════════════════════
   void run()
   {
      switch (currentState)
      {
      case ElevatorState::REST:
         leftElevator.move(0);
         rightElevator.move(0);
         return;
         break;
      case ElevatorState::STOW:
         if(isElevatorSettled()) {
            setWristTarget(ElevatorConfig::STOW_WRIST);
         }
         else if(fabs(getWristPosition() - ElevatorConfig::STOW_WRIST) < ElevatorConfig::WRIST_SETTLE_DEG) {
            setState(ElevatorState::MANUAL);
         }
         break;
      case ElevatorState::SCORE:
         if(fabs(wristTarget - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
            elevatorTarget = ElevatorConfig::SCORE_ELEV;
         }
         if(elevatorTarget == ElevatorConfig::SCORE_ELEV) {
            if(isElevatorStalled()) {
               setState(ElevatorState::REST);
               reachedSpeed = false;
            }
         }
         break;
      case ElevatorState::RESET:
         if(fabs(wristTarget - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
            elevatorTarget = -100;
         }
         if(elevatorTarget == -100) {
            if(isElevatorStalled()) {
               setState(ElevatorState::LOAD);
               reachedSpeed = false;
               elevatorReset();
            }
         }
         break;
      case ElevatorState::PICKUP:
         // if(isElevatorSettled()) {
         //    setWristTarget(ElevatorConfig::PICKUP_WRIST);
         // }
         // if(fabs(getWristPosition() - ElevatorConfig::PICKUP_WRIST) < ElevatorConfig::WRIST_SETTLE_DEG) {
         //    elevatorTarget = -100;
         // }
         // if(elevatorTarget == -100) {
         //    if(isElevatorStalled()) {
         //       controller.rumble(".");
         //       pros::delay(300);
         //       clamp.setState(true);
         //       pros::delay(300);
         //       setState(ElevatorState::STOW);
         //    }            
         // }
         switch(pickUpPhase) {
            case(0):
               if (isElevatorSettled()) {
                  setWristTarget(ElevatorConfig::PICKUP_WRIST);
                  pickUpPhase = 1;
               }
               break;
            case(1):
               if (isWristSettled()) {
                  elevatorTarget = 1;
                  startTime = pros::millis();
                  pickUpPhase = 2;
               }
               break;
            case(2):
               if(isElevatorStalled()) {
                  leftElevator.move(0);
                  rightElevator.move(0);
                  clamp.setState(true);
                  pros::delay(300);
                  setState(ElevatorState::STOW);
                  return;
               }
               break;
            
         } 
         break;
      }
      double elevPos = getElevatorPosition();
      double wristPos = getWristPosition();

      double elevErr = elevatorTarget - elevPos;
      double wristErr = wristTarget - wristPos;

      double elevOut = elevatorPID.update(elevErr);
      double wristOut = wristPID.update(wristErr);

      leftPower = clamp127(-wristOut + elevOut);
      rightPower = clamp127(-wristOut - elevOut);
      printf("leftPower:%d\n ", leftPower);
      printf("rightPower:%d\n ", rightPower);
      printf("elevatorPosition:%d\n ", int(elevPos));
      printf("wristPosition:%d\n ", int(wristPos));
      if (isSettled()) {
         leftPower = 0;
         rightPower = 0;
      }
      // if (isStalled()){
      //    leftPower = 0;
      //    rightPower = 0;
      // }
      

      leftElevator.move(leftPower);
      rightElevator.move(rightPower);
   }
};

inline Elevator elevator;

#endif