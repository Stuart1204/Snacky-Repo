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
   constexpr double STOW_ELEV = 0.0, STOW_WRIST = 0.0;
   constexpr double SCORE_LOW_ELEV = 8.0, SCORE_LOW_WRIST = 30.0;
   constexpr double SCORE_HIGH_ELEV = 20.0, SCORE_HIGH_WRIST = 45.0;

   // PID gains  { kP, kI, kD, windupRange }
   constexpr double ELEV_KP = 45, ELEV_KI = 0.0, ELEV_KD = 0;
   constexpr double WRIST_KP = 1.5, WRIST_KI = 0.0, WRIST_KD = 1;

   // Settle threshold
   constexpr double ELEV_SETTLE_IN = 0.15;  // inches
   constexpr double WRIST_SETTLE_DEG = 1.5; // degrees
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
   int currentState = ElevatorState::STOW;
   int lastState = ElevatorState::STOW;
   double elevatorTarget = 0.0; // inches
   double wristTarget = 0.0;    // degrees

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

      switch (currentState)
      {
      case ElevatorState::STOW:
         applyMacro(ElevatorState::STOW,
                    ElevatorConfig::STOW_ELEV,
                    ElevatorConfig::STOW_WRIST);
         break;

      case ElevatorState::SCORE_LOW:
         applyMacro(ElevatorState::SCORE_LOW,
                    ElevatorConfig::SCORE_LOW_ELEV,
                    ElevatorConfig::SCORE_LOW_WRIST);
         break;

      case ElevatorState::SCORE_HIGH:
         applyMacro(ElevatorState::SCORE_HIGH,
                    ElevatorConfig::SCORE_HIGH_ELEV,
                    ElevatorConfig::SCORE_HIGH_WRIST);
         break;

      case ElevatorState::MANUAL:
         holdCurrent();
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
      return wrist_rotation.get_position() / 100.0;
   }

   void changeWristTarget(double delta)
   {
      wristTarget = wristTarget + delta;
   }

   // Direct target setters — use in autonomous
   void setElevatorTarget(double inches)
   {
      elevatorTarget = clampD(inches,
                              ElevatorConfig::ELEVATOR_MIN,
                              ElevatorConfig::ELEVATOR_MAX);
   }

   void setWristTarget(double degrees)
   {
      wristTarget = clampD(degrees,
                           ElevatorConfig::WRIST_MIN,
                           ElevatorConfig::WRIST_MAX);
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
      wrist_rotation.reset_position();
      elevatorTarget = 0;
      wristTarget = 0;
      elevatorPID.reset();
      wristPID.reset();
      currentState = ElevatorState::STOW;
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
   }
};

inline Elevator elevator;

#endif