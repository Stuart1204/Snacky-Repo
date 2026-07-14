#ifndef ELEVATOR_CPP
#define ELEVATOR_CPP

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
namespace ElevatorConfig
{
   // Encoder ticks (motor degrees) per inch of elevator travel.
   //
   constexpr double TICKS_PER_INCH = 62.5;

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
   constexpr double STOW_ELEV = 17.0, STOW_WRIST = -5.0;
   constexpr double SCORE_ELEV = -10, SCORE_WRIST = -70.0;
   constexpr double RESET_ELEV = 7, RESET_WRIST = -80.0;
   constexpr double LOAD_ELEV = 3, LOAD_WRIST = -80.0;
   constexpr double PICKUP_ELEV = 15, PICKUP_WRIST = -200.0;

   // PID gains  { kP, kI, kD, windupRange }
   constexpr double ELEV_KP = 45, ELEV_KI = 0.0, ELEV_KD = 0;
   constexpr double WRIST_KP = 2, WRIST_KI = 0.0, WRIST_KD = 1;

   // Settle threshold
   constexpr double ELEV_SETTLE_IN = 3;  // inches
   constexpr double WRIST_SETTLE_DEG = 10; // degrees
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
   double elevatorTarget = 0.0; // inches
   double wristTarget = 0.0;    // degrees
   bool reachedSpeed = false;
   int reachedSpeedThreshold = 200;

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
      // leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      // rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
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
         setWristTarget(ElevatorConfig::STOW_WRIST);
         setElevatorTarget(ElevatorConfig::STOW_ELEV);
         break;
      case ElevatorState::SCORE:
         setWristTarget(ElevatorConfig::SCORE_WRIST);
         elevatorTarget = getElevatorPosition();
         reachedSpeed = false;
         break;
      case ElevatorState::REST:
         // setElevatorTarget(getElevatorPosition());
         // setWristTarget(getWristPosition());
         break;
      case ElevatorState::RESET:
         setWristTarget(ElevatorConfig::RESET_WRIST);
         setElevatorTarget(ElevatorConfig::RESET_ELEV);
         reachedSpeed = false;
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
         reachedSpeed = false;
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

   // Blocking wait — call in autonomous tasks only
   void waitUntilSettled(uint32_t timeoutMs = 2000)
   {
      uint32_t start = pros::millis();
      while (!isSettled() && (pros::millis() - start) < timeoutMs)
      {
         pros::delay(10);
      }
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
         break;
      case ElevatorState::SCORE:
         if(fabs(wristTarget - getWristPosition()) < 15) {
            elevatorTarget = ElevatorConfig::SCORE_ELEV;
         }
         if(elevatorTarget == ElevatorConfig::SCORE_ELEV) {
            if(reachedSpeed == false && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 > reachedSpeedThreshold) {
            reachedSpeed = true;
            }
            if(reachedSpeed && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 < reachedSpeedThreshold) {
               setState(ElevatorState::REST);
               reachedSpeed = false;
            }
         }
         break;
      case ElevatorState::RESET:
         if(fabs(wristTarget - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
            elevatorTarget = -20;
         }
         if(elevatorTarget == -20) {
            if(reachedSpeed == false && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 > reachedSpeedThreshold) {
            reachedSpeed = true;
            }
            if(reachedSpeed && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 < reachedSpeedThreshold) {
               setState(ElevatorState::LOAD);
               reachedSpeed = false;
               elevatorReset();
            }
         }
         break;
      case ElevatorState::PICKUP:
         if(fabs(elevatorTarget - getElevatorPosition()) < ElevatorConfig::ELEV_SETTLE_IN) {
            setWristTarget(ElevatorConfig::PICKUP_WRIST);
         }
         if(fabs(ElevatorConfig::PICKUP_WRIST - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
            elevatorTarget = 4;
         }
         if(elevatorTarget == 4) {
            if(reachedSpeed == false && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 > reachedSpeedThreshold) {
            reachedSpeed = true;
            }
            if(reachedSpeed && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 < reachedSpeedThreshold) {
               reachedSpeed = false;
               clamp.setState(true);
               pros::delay(500);
               setState(ElevatorState::STOW);
            }
         }
         break;
      }

      double elevPos = getElevatorPosition();
      double wristPos = getWristPosition();

      double elevErr = elevatorTarget - elevPos;
      double wristErr = wristTarget - wristPos;

      double elevOut = elevatorPID.update(elevErr);
      double wristOut = wristPID.update(wristErr);

      int leftPower = clamp127(wristOut + elevOut);
      int rightPower = clamp127(wristOut - elevOut);
      printf("leftPower:%d\n ", leftPower);
      printf("rightPower:%d\n ", rightPower);
      printf("elevatorPosition:%d\n ", int(elevPos));
      printf("wristPosition:%d\n ", int(wristPos));

      leftElevator.move(leftPower);
      rightElevator.move(rightPower);

      pros::screen::print(TEXT_LARGE_CENTER, 1, "leftPower: %f", leftPower);
      pros::screen::print(TEXT_LARGE_CENTER, 3, "rightPower: %f", rightPower);
   }
};

inline Elevator elevator;

#endif