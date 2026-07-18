// #ifndef ELEVATOR_CPP
// #define ELEVATOR_CPP

// #include "globals.hpp"
// #include "main.h"
// #include "piston.cpp"
// #include "pros/adi.hpp"
// #include "pros/misc.hpp"
// #include "pros/rtos.hpp"
// namespace ElevatorConfig
// {
//    // Encoder ticks (motor degrees) per inch of elevator travel.
//    //
//    constexpr double TICKS_PER_INCH = 57.5;

//    // Wrist rotation sensor: degrees reported per degree of actual wrist rotation.
//    constexpr double SENSOR_DEG_PER_WRIST_DEG = 1.0;

//    // Elevator soft limits (inches)
//    constexpr double ELEVATOR_MIN = 0.0;
//    constexpr double ELEVATOR_MAX = 40.0;

//    // Wrist soft limits (degrees from home)
//    constexpr double WRIST_MIN = -90.0;
//    constexpr double WRIST_MAX = 90.0;

//    // ── Macro target positions ──────────────────────────────
//    // { elevatorInches, wristDegrees }
//    constexpr double STOW_ELEV = 15.0, STOW_WRIST = -5.0;
//    constexpr double SCORE_ELEV = -10, SCORE_WRIST = -70.0;
//    constexpr double RESET_ELEV = 7, RESET_WRIST = -80.0;
//    constexpr double LOAD_ELEV = 1.4, LOAD_WRIST = -80.0;
//    constexpr double MATCHLOAD_ELEV = 5, MATCHLOAD_WRIST = -140.0;
//    constexpr double PICKUP_ELEV = 10, PICKUP_WRIST = -189.0;
//    constexpr double MANUAL_ELEV = 10, MANUAL_WRIST = -5.0;

//    // PID gains  { kP, kI, kD, windupRange }
//    constexpr double ELEV_KP = 45, ELEV_KI = 0.0, ELEV_KD = 0;
//    constexpr double WRIST_KP = 4.5, WRIST_KI = 0.0, WRIST_KD = 1.7;

//    // Settle threshold
//    constexpr double ELEV_SETTLE_IN = 2.5;  // inches
//    constexpr double WRIST_SETTLE_DEG = 20; // degrees
// }

// class Elevator
// {
// private:
//    lemlib::PID elevatorPID{
//        ElevatorConfig::ELEV_KP,
//        ElevatorConfig::ELEV_KI,
//        ElevatorConfig::ELEV_KD, 0};
//    lemlib::PID wristPID{
//        ElevatorConfig::WRIST_KP,
//        ElevatorConfig::WRIST_KI,
//        ElevatorConfig::WRIST_KD, 0};
//    int currentState = ElevatorState::REST;
//    int lastState = ElevatorState::REST;
//    int pickUpPhase = 0;
//    double elevatorTarget = 0.0; // inches
//    double wristTarget = 0.0;    // degrees
//    bool reachedSpeed = false;
//    int reachedSpeedThreshold = 200;
//    uint32_t startTime = 0;

//    int leftPower = 0;
//    int rightPower = 0;

//    static double clampD(double v, double lo, double hi)
//    {
//       if (v < lo)
//          return lo;
//       if (v > hi)
//          return hi;
//       return v;
//    }
//    static int clamp127(double v) {
//       if (v >  127) return  127;
//       if (v < -127) return -127;
//       return static_cast<int>(v);
//    }

//    // Apply a macro target pair and transition state
//    void applyMacro(int newState, double elevIn, double wristDeg)
//    {
//       lastState = currentState;
//       currentState = newState;
//       elevatorTarget = clampD(elevIn, ElevatorConfig::ELEVATOR_MIN,
//                               ElevatorConfig::ELEVATOR_MAX);
//       wristTarget = clampD(wristDeg, ElevatorConfig::WRIST_MIN,
//                            ElevatorConfig::WRIST_MAX);
//       elevatorPID.reset();
//       wristPID.reset();
//    }

// public:
//    Elevator()
//    {
//       leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//       rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
//    }

//    void setState(int state)
//    {
//       if (state == currentState)
//          return;

//       lastState = currentState;
//       currentState = state;

//       leftElevator.move(0);
//       rightElevator.move(0);

//       switch (currentState)
//       {
//       case ElevatorState::STOW:
//          setWristTarget(-180);
//          setElevatorTarget(ElevatorConfig::STOW_ELEV);
//          break;
//       case ElevatorState::SCORE:
//          setWristTarget(ElevatorConfig::SCORE_WRIST);
//          elevatorTarget = getElevatorPosition();
//          reachedSpeed = false;
//          startTime = pros::millis();
//          break;
//       case ElevatorState::REST:
//          // setElevatorTarget(getElevatorPosition());
//          // setWristTarget(getWristPosition());
//          break;
//       case ElevatorState::RESET:
//          setWristTarget(ElevatorConfig::RESET_WRIST);
//          setElevatorTarget(getElevatorPosition());
//          reachedSpeed = false;
//          startTime = pros::millis();
//          break;
//       case ElevatorState::LOAD:
//          setWristTarget(ElevatorConfig::LOAD_WRIST);
//          setElevatorTarget(ElevatorConfig::LOAD_ELEV);
//          reachedSpeed = false;
//          break;
//       case ElevatorState::PICKUP:
//          setWristTarget(getWristPosition());
//          setElevatorTarget(ElevatorConfig::PICKUP_ELEV);
//          clamp.setState(false);
//          pickUpPhase = 0;
//          reachedSpeed = false;
//          break;
//       case ElevatorState::MANUAL:
//          setWristTarget(ElevatorConfig::MANUAL_WRIST);
//       case ElevatorState::MATCHLOAD:
//          setWristTarget(ElevatorConfig::MATCHLOAD_WRIST);
//          setElevatorTarget(ElevatorConfig::MATCHLOAD_ELEV);
//          break;
//       }
//    }
//    int getState() const { return currentState; }

//    // ══════════════════════════════════════════════════════
//    //  MANUAL INCREMENT CONTROLS
//    //  Call these from opcontrol button presses.
//    //  They work in MANUAL state; in macro states they also
//    //  nudge the target (useful for fine-tuning after a macro).
//    // ══════════════════════════════════════════════════════

//    void changeElevatorTarget(double delta)
//    {
//       elevatorTarget = elevatorTarget + delta;
//    }

//    double getElevatorTarget()
//    {
//       return elevatorTarget;
//    }
//    double getWristTarget()
//    {
//       return wristTarget;
//    }
//    double getElevatorPosition()
//    {
//       double elevTicks = (leftElevator.get_position() - rightElevator.get_position()) / 2.0;
//       return elevTicks / ElevatorConfig::TICKS_PER_INCH;
//    }

//    double getWristPosition()
//    {
//       double pos = wristRotation.get_angle() / 100.0;
//       if(pos > 45) {
//          pos -= 360;
//       }
//       return pos;
//    }

//    void changeWristTarget(double delta)
//    {
//       wristTarget = wristTarget + delta;
//    }

//    // Direct target setters — use in autonomous
//    void setElevatorTarget(double inches)
//    {
//       elevatorTarget = inches;
//    }

//    void setWristTarget(double degrees)
//    {
//       if(degrees > 45) {
//          wristTarget = degrees - 360;
//       } else {
//          wristTarget = degrees;
//       }
//    }

//    // Freeze targets at current sensor readings
//    void holdCurrent()
//    {
//       elevatorTarget = getElevatorPosition();
//       wristTarget = getWristPosition();
//       elevatorPID.reset();
//       wristPID.reset();
//    }

//    // Tare everything back to zero (use when physically at home)
//    void elevatorReset()
//    {
//       leftElevator.tare_position();
//       rightElevator.tare_position();
//       elevatorPID.reset();
//       wristPID.reset();
//    }

//    // ── Settle check (useful in auton waits) ───────────────
//    bool isElevatorSettled()
//    {
//       return std::abs(getElevatorPosition() - elevatorTarget) < ElevatorConfig::ELEV_SETTLE_IN;
//    }

//    bool isWristSettled()
//    {
//       return std::abs(getWristPosition() - wristTarget) < ElevatorConfig::WRIST_SETTLE_DEG;
//    }

//    bool isSettled()
//    {
//       return isElevatorSettled() && isWristSettled();
//    }

//    bool isElevatorStalled() {
//       // if(reachedSpeed == false && (pros::millis() - startTime > 1000 || fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 > reachedSpeedThreshold)) {
//       //    reachedSpeed = true;
//       // }
//       // if(reachedSpeed && fabs(fabs(leftElevator.get_actual_velocity()) + fabs(leftElevator.get_actual_velocity())) / 2 < reachedSpeedThreshold) {
//       //    reachedSpeed = false;
//       //    return true;
//       // }
//       // return false;
//       double avgVelocity = (fabs(leftElevator.get_actual_velocity()) + fabs(rightElevator.get_actual_velocity())) / 2;
//       double avgCurrent = (fabs(leftElevator.get_current_draw())+fabs(rightElevator.get_current_draw())) / 2;
//       if (pros::millis() - startTime < 200) {
//          return false;
//       }
//       return ((avgVelocity < 10 && avgCurrent > 1800) || pros::millis() - startTime > 1000);
//    }

//    bool isStalled() {
//       static uint32_t noProgressStart = 0;
//       static double lastPos = 0;
//       double currentPos = getElevatorPosition();
//       bool isMoving = fabs(currentPos - lastPos) > 0.2;
//       if (isMoving) {
//          noProgressStart = pros::millis();
//          lastPos = currentPos;
//          return false;
//       }
//       return (pros::millis()-noProgressStart)>2000;
//    }

//    // Blocking wait — call in autonomous tasks only
//    void waitUntilSettled(uint32_t timeoutMs = 2000)
//    {
//       uint32_t start = pros::millis();
//       while (!isSettled() && (pros::millis() - start) < timeoutMs)
//       {
//          pros::delay(10);
//       }
//    }

//    int getLeftPower() {
//       return leftPower;
//    }

//    int getRightPower() {
//       return rightPower;
//    }

//    // ══════════════════════════════════════════════════════
//    //  RUN  — called every ~10 ms from a dedicated thread
//    // ══════════════════════════════════════════════════════
//    void run()
//    {
//       switch (currentState)
//       {
//       case ElevatorState::REST:
//          leftElevator.move(0);
//          rightElevator.move(0);
//          return;
//          break;
//       case ElevatorState::STOW:
//          if(isElevatorSettled()) {
//             setWristTarget(ElevatorConfig::STOW_WRIST);
//          }
//          else if(fabs(getWristPosition() - ElevatorConfig::STOW_WRIST) < ElevatorConfig::WRIST_SETTLE_DEG) {
//             setState(ElevatorState::MANUAL);
//          }
//          break;
//       case ElevatorState::SCORE:
//          if(fabs(wristTarget - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
//             elevatorTarget = ElevatorConfig::SCORE_ELEV;
//          }
//          if(elevatorTarget == ElevatorConfig::SCORE_ELEV) {
//             if(isElevatorStalled()) {
//                setState(ElevatorState::REST);
//                reachedSpeed = false;
//             }
//          }
//          break;
//       case ElevatorState::RESET:
//          if(fabs(wristTarget - getWristPosition()) < ElevatorConfig::WRIST_SETTLE_DEG) {
//             elevatorTarget = -100;
//          }
//          if(elevatorTarget == -100) {
//             if(isElevatorStalled()) {
//                setState(ElevatorState::LOAD);
//                reachedSpeed = false;
//                elevatorReset();
//             }
//          }
//          break;
//       case ElevatorState::PICKUP:
//          // if(isElevatorSettled()) {
//          //    setWristTarget(ElevatorConfig::PICKUP_WRIST);
//          // }
//          // if(fabs(getWristPosition() - ElevatorConfig::PICKUP_WRIST) < ElevatorConfig::WRIST_SETTLE_DEG) {
//          //    elevatorTarget = -100;
//          // }
//          // if(elevatorTarget == -100) {
//          //    if(isElevatorStalled()) {
//          //       controller.rumble(".");
//          //       pros::delay(300);
//          //       clamp.setState(true);
//          //       pros::delay(300);
//          //       setState(ElevatorState::STOW);
//          //    }
//          // }
//          switch(pickUpPhase) {
//             case(0):
//                if (isElevatorSettled()) {
//                   setWristTarget(ElevatorConfig::PICKUP_WRIST);
//                   pickUpPhase = 1;
//                }
//                break;
//             case(1):
//                if (isWristSettled()) {
//                   elevatorTarget = 1;
//                   startTime = pros::millis();
//                   pickUpPhase = 2;
//                }
//                break;
//             case(2):
//                if(isElevatorStalled()) {
//                   leftElevator.move(0);
//                   rightElevator.move(0);
//                   clamp.setState(true);
//                   pros::delay(300);
//                   setState(ElevatorState::STOW);
//                   return;
//                }
//                break;

//          }
//          break;
//       }
//       double elevPos = getElevatorPosition();
//       double wristPos = getWristPosition();

//       double elevErr = elevatorTarget - elevPos;
//       double wristErr = wristTarget - wristPos;

//       double elevOut = elevatorPID.update(elevErr);
//       double wristOut = wristPID.update(wristErr);

//       leftPower = clamp127(-wristOut + elevOut);
//       rightPower = clamp127(-wristOut - elevOut);
//       printf("leftPower:%d\n ", leftPower);
//       printf("rightPower:%d\n ", rightPower);
//       printf("elevatorPosition:%d\n ", int(elevPos));
//       printf("wristPosition:%d\n ", int(wristPos));
//       if (isSettled()) {
//          leftPower = 0;
//          rightPower = 0;
//       }
//       // if (isStalled()){
//       //    leftPower = 0;
//       //    rightPower = 0;
//       // }

//       leftElevator.move(leftPower);
//       rightElevator.move(rightPower);
//    }
// };

// inline Elevator elevator;

// #endif

#pragma once

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

// ═══════════════════════════════════════════════════════════
//  CONFIGURATION
// ═══════════════════════════════════════════════════════════
namespace ElevatorConfig
{
   // Mechanical conversion
   constexpr double TICKS_PER_INCH = 83;
   constexpr double SENSOR_DEG_PER_WRIST_DEG = 1.0;

   // Soft limits
   constexpr double ELEVATOR_MIN = -200.0; // generous to allow stall-seeking
   constexpr double ELEVATOR_MAX = 40.0;
   constexpr double WRIST_MIN = -300.0;
   constexpr double WRIST_MAX = 90.0;

   // ── Macro target positions { elevatorInches, wristDegrees } ──
   constexpr double STOW_ELEV = 15.0;
   constexpr double STOW_WRIST = -5.0;

   constexpr double SCORE_ELEV = -10.0;
   constexpr double SCORE_WRIST = -70.0;

   constexpr double RESET_ELEV = 10;
   constexpr double RESET_WRIST = -90.0;      // level wrist before driving down
   constexpr double RESET_ELEV_SEEK = -100.0; // drive into hard stop

   constexpr double LOAD_ELEV = 1.4;
   constexpr double LOAD_WRIST = -80.0;

   constexpr double MATCHLOAD_ELEV = 5.0;
   constexpr double MATCHLOAD_WRIST = -140.0;

   // Pickup sequence targets
   constexpr double PICKUP_APPROACH_ELEV = 10.0;    // phase 0: move above game piece
   constexpr double PICKUP_VERTICAL_WRIST = -175.0; // phase 1: wrist vertical
   constexpr double PICKUP_SEEK_ELEV = 2;           // phase 2: drive down to stall
   constexpr double PICKUP_SWEEP_WRIST = -170.0;    // phase 3: sweep to consolidate

   constexpr double MANUAL_WRIST = -5.0;

   // PID gains { kP, kI, kD }
   constexpr double ELEV_KP = 45.0, ELEV_KI = 0.0, ELEV_KD = 0.0;
   constexpr double WRIST_KP = 3, WRIST_KI = 0.0, WRIST_KD = 1.7;

   // Settle thresholds
   constexpr double ELEV_SETTLE_IN = 2.5;    // inches
   constexpr double WRIST_SETTLE_DEG = 15.0; // degrees

   // Stall detection
   constexpr double STALL_VELOCITY_THRESHOLD = 20.0;  // RPM
   constexpr double STALL_CURRENT_THRESHOLD = 1800.0; // mA
   constexpr uint32_t STALL_GRACE_PERIOD_MS = 300;    // ignore stall during acceleration
   constexpr uint32_t STALL_TIMEOUT_MS = 1000;        // max time before forced stop

   // Global stall protection (overheating prevention)
   constexpr uint32_t GLOBAL_STALL_DURATION_MS = 500; // stall this long → emergency stop
   constexpr double GLOBAL_STALL_VELOCITY = 20.0;
   constexpr double GLOBAL_STALL_CURRENT = 2000.0;

   // Clamp delay
   constexpr uint32_t CLAMP_ACTIVATE_DELAY_MS = 300;
}

// ═══════════════════════════════════════════════════════════
//  ELEVATOR CLASS
// ═══════════════════════════════════════════════════════════
class Elevator
{
private:
   // ── PID controllers ──
   lemlib::PID elevatorPID{
       ElevatorConfig::ELEV_KP,
       ElevatorConfig::ELEV_KI,
       ElevatorConfig::ELEV_KD, 0};
   lemlib::PID wristPID{
       ElevatorConfig::WRIST_KP,
       ElevatorConfig::WRIST_KI,
       ElevatorConfig::WRIST_KD, 0};

   // ── State tracking ──
   int currentState = ElevatorState::REST;
   int lastState = ElevatorState::REST;
   int phase = 0;              // sub-phase within a macro state
   bool stateComplete = false; // set true when a macro finishes its sequence

   // ── Targets ──
   double elevatorTarget = 0.0;
   double wristTarget = 0.0;

   // ── Motor output (for telemetry) ──
   int leftPower = 0;
   int rightPower = 0;

   // ── Stall detection (per-state) ──
   uint32_t motionStartTime = 0; // when current motion phase began
   uint32_t stallStartTime = 0;  // when stall condition first detected

   // ── Global stall protection ──
   uint32_t globalStallStart = 0;
   bool globalStallActive = false;

   // ── Sensor-assisted pickup (for auton) ──
   bool useSensorPickup = false;
   std::function<bool()> sensorDetectFunc = nullptr; // returns true when game piece detected

   // ═══════════════════════════════════════════════════════
   //  UTILITY
   // ═══════════════════════════════════════════════════════
   static double clampD(double v, double lo, double hi)
   {
      return (v < lo) ? lo : (v > hi) ? hi
                                      : v;
   }

   static int clamp127(double v)
   {
      return static_cast<int>(clampD(v, -127.0, 127.0));
   }

   // ═══════════════════════════════════════════════════════
   //  STALL DETECTION
   //  Returns true if motors are stalled (high current, no movement).
   //  Respects a grace period after motionStartTime to allow acceleration.
   // ═══════════════════════════════════════════════════════
   bool isMotionStalled()
   {
      // Grace period: don't report stall while motors are still accelerating
      if (pros::millis() - motionStartTime < ElevatorConfig::STALL_GRACE_PERIOD_MS)
      {
         stallStartTime = pros::millis();
         return false;
      }

      // Hard timeout: if motion takes way too long, declare stall
      if (pros::millis() - motionStartTime > ElevatorConfig::STALL_TIMEOUT_MS)
      {
         return true;
      }

      double avgVelocity = (fabs(leftElevator.get_actual_velocity()) +
                            fabs(rightElevator.get_actual_velocity())) /
                           2.0;
      double avgCurrent = (fabs(leftElevator.get_current_draw()) +
                           fabs(rightElevator.get_current_draw())) /
                          2.0;

      bool stalledNow = (avgVelocity < ElevatorConfig::STALL_VELOCITY_THRESHOLD &&
                         avgCurrent > ElevatorConfig::STALL_CURRENT_THRESHOLD);

      if (!stalledNow)
      {
         stallStartTime = pros::millis();
      }

      // Require sustained stall for at least 100ms to filter noise
      return (pros::millis() - stallStartTime > 100);
   }

   // ═══════════════════════════════════════════════════════
   //  GLOBAL STALL PROTECTION
   //  Prevents motor overheating regardless of state.
   //  If motors are commanded hard but not moving for too long, ESTOP.
   // ═══════════════════════════════════════════════════════
   bool checkGlobalStallProtection()
   {
      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
         return false;

      double avgVelocity = (fabs(leftElevator.get_actual_velocity()) +
                            fabs(rightElevator.get_actual_velocity())) /
                           2.0;
      double avgCurrent = (fabs(leftElevator.get_current_draw()) +
                           fabs(rightElevator.get_current_draw())) /
                          2.0;
      bool commandingHard = (abs(leftPower) > 70 || abs(rightPower) > 70);

      bool stalledNow = commandingHard &&
                        avgVelocity < ElevatorConfig::GLOBAL_STALL_VELOCITY &&
                        avgCurrent > ElevatorConfig::GLOBAL_STALL_CURRENT;

      if (stalledNow)
      {
         if (!globalStallActive)
         {
            globalStallActive = true;
            globalStallStart = pros::millis();
         }
         else if (pros::millis() - globalStallStart > ElevatorConfig::GLOBAL_STALL_DURATION_MS)
         {
            return true; // emergency stop
         }
      }
      else
      {
         globalStallActive = false;
      }
      return false;
   }

   // ═══════════════════════════════════════════════════════
   //  Begin a new motion phase — resets stall timers
   // ═══════════════════════════════════════════════════════
   void beginMotion()
   {
      motionStartTime = pros::millis();
      stallStartTime = pros::millis();
      elevatorPID.reset();
      wristPID.reset();
   }

   // ═══════════════════════════════════════════════════════
   //  PHASE RUNNERS — one per macro state
   // ═══════════════════════════════════════════════════════

   void runStow()
   {
      // Phase 0: Move elevator to stow height, wrist safe
      // Phase 1: Once elevator settled, move wrist to stow
      // Phase 2: Settled → complete
      switch (phase)
      {
      case 0:
         elevatorTarget = ElevatorConfig::STOW_ELEV;
         wristTarget = -180.0; // safe transit angle
         if (isElevatorSettled())
         {
            phase = 1;
         }
         break;
      case 1:
         wristTarget = ElevatorConfig::STOW_WRIST;
         if (isWristSettled())
         {
            phase = 2;
            stateComplete = true;
         }
         break;
      case 2:
         // Hold position, state complete
         break;
      }
   }

   void runScore()
   {
      // Phase 0: Move wrist to score angle
      //          - Below 20": single-motor wrist rotation (no elevator drift)
      //          - Above 20": normal differential wrist control
      // Phase 1: Drive elevator down until stall
      // Phase 2: Release clamp, complete
      switch (phase)
      {
      case 0:
         if (getElevatorPosition() < 20.0)
         {
            wristTarget = ElevatorConfig::SCORE_WRIST;
            // Override PID output — only spin left motor for pure wrist rotation
            double wristErr = wristTarget - getWristPosition();
            double wristOut = wristPID.update(wristErr);
            leftPower = clamp127(-wristOut);
            rightPower = 0;
            leftElevator.move(leftPower);
            rightElevator.move(0);

            if (isWristSettled())
            {
               elevatorTarget = ElevatorConfig::SCORE_ELEV;
               beginMotion();
               phase = 1;
            }
            return; // skip the normal PID output at the end of run()
         }
         else
         {
            wristTarget = ElevatorConfig::SCORE_WRIST;
            // High enough — normal differential is fine
            if (isWristSettled())
            {
               elevatorTarget = ElevatorConfig::SCORE_ELEV;
               beginMotion();
               phase = 1;
            }
         }
         break;
      case 1:
         if (isMotionStalled())
         {
            clamp.setState(false);
            phase = 2;
            stateComplete = true;
            setState(ElevatorState::REST);
         }
         break;
      }
   }

   void runReset()
   {
      // Phase 0: Raise elevator to clear funnel (hold current wrist angle)
      // Phase 1: Level the wrist
      // Phase 2: Drive elevator down until stalled on hard stop
      // Phase 3: Reset encoders, transition to LOAD
      switch (phase)
      {
      case 0:
         elevatorTarget = ElevatorConfig::RESET_ELEV;
         wristTarget = getWristPosition(); // explicitly hold current wrist angle
         if (isElevatorSettled())
            phase = 1;
         break;
      case 1:
         wristTarget = ElevatorConfig::RESET_WRIST;
         if (isWristSettled())
         {
            elevatorTarget = ElevatorConfig::RESET_ELEV_SEEK;
            beginMotion();
            phase = 2;
         }
         break;
      case 2:
         if (isMotionStalled())
            phase = 3;
         break;
      case 3:
         leftElevator.move(0);
         rightElevator.move(0);
         elevatorReset();
         stateComplete = true;
         setState(ElevatorState::LOAD);
         break;
      }
   }

   void runLoad()
   {
      // Simple: move to load position and hold
      elevatorTarget = ElevatorConfig::LOAD_ELEV;
      wristTarget = ElevatorConfig::LOAD_WRIST;
      if (isSettled())
      {
         stateComplete = true;
      }
   }

   void runMatchload()
   {
      elevatorTarget = ElevatorConfig::MATCHLOAD_ELEV;
      wristTarget = ElevatorConfig::MATCHLOAD_WRIST;
      if (isSettled())
      {
         stateComplete = true;
      }
   }

   void runPickup()
   {
      // Phase 0: Move elevator to approach height
      // Phase 1: Rotate wrist vertical, then WAIT for game piece detection
      // Phase 2: Drive elevator down until stall on game piece
      // Phase 3: Sweep wrist to consolidate
      // Phase 4: Activate clamp, wait, then transition to STOW
      switch (phase)
      {
      case 0:
         elevatorTarget = ElevatorConfig::PICKUP_APPROACH_ELEV;
         if (isElevatorSettled())
         {
            wristTarget = ElevatorConfig::PICKUP_VERTICAL_WRIST;
            phase = 1;
         }
         break;
      case 1:
      {
         // Wrist must be vertical before we start waiting
         if (!isWristSettled())
            break;

         // Determine if we should begin descent
         bool shouldDescend = false;

         if (useSensorPickup && sensorDetectFunc != nullptr)
         {
            // Auton: wait for sensor to detect game piece
            shouldDescend = sensorDetectFunc();
         }
         else
         {
            // Driver: descend immediately once wrist is ready
            // (driver already committed by pressing the button)
            shouldDescend = true;
         }

         if (shouldDescend)
         {
            elevatorTarget = ElevatorConfig::PICKUP_SEEK_ELEV;
            beginMotion();
            phase = 2;
         }
         break;
      }
      case 2:
         if (isElevatorSettled() || isMotionStalled())
         {
            elevatorTarget = getElevatorPosition();
            wristTarget = ElevatorConfig::PICKUP_SWEEP_WRIST;
            phase = 3;
         }
         break;
      case 3:
         if (isWristSettled())
         {
            clamp.setState(true);
            motionStartTime = pros::millis();
            phase = 4;
         }
         break;
      case 4:
         if (pros::millis() - motionStartTime > ElevatorConfig::CLAMP_ACTIVATE_DELAY_MS)
         {
            stateComplete = true;
            setState(ElevatorState::STOW);
         }
         break;
      }
   }

   void runManual()
   {
      // Elevator target is set externally by driver controls.
      // Wrist holds manual position.
      wristTarget = ElevatorConfig::MANUAL_WRIST;
   }

public:
   // ═══════════════════════════════════════════════════════
   //  CONSTRUCTOR
   // ═══════════════════════════════════════════════════════
   Elevator()
   {
      leftElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      rightElevator.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
   }

   // ═══════════════════════════════════════════════════════
   //  STATE MANAGEMENT
   // ═══════════════════════════════════════════════════════
   void setState(int state)
   {
      if (state == currentState)
         return;

      lastState = currentState;
      currentState = state;
      phase = 0;
      stateComplete = false;

      // Stop motors during transition
      leftElevator.move(0);
      rightElevator.move(0);

      // Reset PID and stall timers
      beginMotion();

      // State-specific initialization
      switch (currentState)
      {
      case ElevatorState::STOW:
         // Phase 0 handles targets
         break;
      case ElevatorState::SCORE:
         elevatorTarget = getElevatorPosition(); // hold until wrist ready
         break;
      case ElevatorState::RESET:
         // Phase 0 handles targets
         break;
      case ElevatorState::LOAD:
         break;
      case ElevatorState::MATCHLOAD:
         break;
      case ElevatorState::PICKUP:
         clamp.setState(false);            // release clamp for pickup
         wristTarget = getWristPosition(); // hold wrist during approach
         break;
      case ElevatorState::MANUAL:
         elevatorTarget = getElevatorPosition();
         wristTarget = ElevatorConfig::MANUAL_WRIST;
         break;
      case ElevatorState::REST:
      case ElevatorState::ESTOP:
         leftElevator.move(0);
         rightElevator.move(0);
         break;
      }
   }

   int getState() const { return currentState; }
   int getLastState() const { return lastState; }
   int getPhase() const { return phase; }
   bool isComplete() const { return stateComplete; }

   // ═══════════════════════════════════════════════════════
   //  SENSOR-ASSISTED PICKUP (AUTON)
   // ═══════════════════════════════════════════════════════
   void enableSensorPickup(std::function<bool()> detectFunc)
   {
      useSensorPickup = true;
      sensorDetectFunc = detectFunc;
   }

   void disableSensorPickup()
   {
      useSensorPickup = false;
      sensorDetectFunc = nullptr;
   }

   // ═══════════════════════════════════════════════════════
   //  POSITION GETTERS
   // ═══════════════════════════════════════════════════════
   double getElevatorPosition()
   {
      double elevTicks = (leftElevator.get_position() - rightElevator.get_position()) / 2.0;
      return elevTicks / ElevatorConfig::TICKS_PER_INCH;
   }

   double getWristPosition()
   {
      double pos = wristRotation.get_angle() / 100.0;
      if (pos > 45.0)
         pos -= 360.0;
      return pos;
   }

   // ═══════════════════════════════════════════════════════
   //  TARGET SETTERS
   // ═══════════════════════════════════════════════════════
   void setElevatorTarget(double inches)
   {
      elevatorTarget = clampD(inches, ElevatorConfig::ELEVATOR_MIN,
                              ElevatorConfig::ELEVATOR_MAX);
   }

   void setWristTarget(double degrees)
   {
      if (degrees > 45.0)
         degrees -= 360.0;
      wristTarget = clampD(degrees, ElevatorConfig::WRIST_MIN,
                           ElevatorConfig::WRIST_MAX);
   }

   double getElevatorTarget() const { return elevatorTarget; }
   double getWristTarget() const { return wristTarget; }

   void changeElevatorTarget(double delta)
   {
      setElevatorTarget(elevatorTarget + delta);
   }

   void changeWristTarget(double delta)
   {
      setWristTarget(wristTarget + delta);
   }

   // ═══════════════════════════════════════════════════════
   //  SETTLE CHECKS
   // ═══════════════════════════════════════════════════════
   bool isElevatorSettled()
   {
      return fabs(getElevatorPosition() - elevatorTarget) < ElevatorConfig::ELEV_SETTLE_IN;
   }

   bool isWristSettled()
   {
      return fabs(getWristPosition() - wristTarget) < ElevatorConfig::WRIST_SETTLE_DEG;
   }

   bool isSettled()
   {
      return isElevatorSettled() && isWristSettled();
   }

   // ═══════════════════════════════════════════════════════
   //  ENCODER RESET
   // ═══════════════════════════════════════════════════════
   void elevatorReset()
   {
      leftElevator.tare_position();
      rightElevator.tare_position();
      elevatorTarget = 0.0;
      elevatorPID.reset();
      wristPID.reset();
   }

   void holdCurrent()
   {
      elevatorTarget = getElevatorPosition();
      wristTarget = getWristPosition();
      elevatorPID.reset();
      wristPID.reset();
   }

   // ═══════════════════════════════════════════════════════
   //  BLOCKING WAITS (auton only)
   // ═══════════════════════════════════════════════════════
   void waitUntilSettled(uint32_t timeoutMs = 3000)
   {
      uint32_t start = pros::millis();
      while (!isSettled() && (pros::millis() - start) < timeoutMs)
         pros::delay(10);
   }

   void waitUntilComplete(uint32_t timeoutMs = 5000)
   {
      uint32_t start = pros::millis();
      while (!stateComplete && (pros::millis() - start) < timeoutMs)
         pros::delay(10);
   }

   void waitUntilPhase(int targetPhase, uint32_t timeoutMs = 3000)
   {
      uint32_t start = pros::millis();
      while (phase < targetPhase && (pros::millis() - start) < timeoutMs)
         pros::delay(10);
   }

   // ═══════════════════════════════════════════════════════
   //  TELEMETRY
   // ═══════════════════════════════════════════════════════
   int getLeftPower() const { return leftPower; }
   int getRightPower() const { return rightPower; }

   void printTelemetry()
   {
      printf("State:%d Phase:%d | Elev pos:%.1f tgt:%.1f | Wrist pos:%.1f tgt:%.1f | L:%d R:%d\n",
             currentState, phase,
             getElevatorPosition(), elevatorTarget,
             getWristPosition(), wristTarget,
             leftPower, rightPower);
   }

   // ═══════════════════════════════════════════════════════
   //  RUN — called every ~10ms from a dedicated thread
   // ═══════════════════════════════════════════════════════
   void run()
   {
      // ── Global stall protection ──
      if (checkGlobalStallProtection())
      {
         leftElevator.move(0);
         rightElevator.move(0);
         leftPower = 0;
         rightPower = 0;
         currentState = ElevatorState::ESTOP;
         return;
      }

      // ── Idle states: zero motors and return ──
      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
      {
         leftElevator.move(0);
         rightElevator.move(0);
         leftPower = 0;
         rightPower = 0;
         return;
      }

      // ── Run the active state's phase logic ──
      switch (currentState)
      {
      case ElevatorState::STOW:
         runStow();
         break;
      case ElevatorState::SCORE:
         runScore();
         break;
      case ElevatorState::RESET:
         runReset();
         break;
      case ElevatorState::LOAD:
         runLoad();
         break;
      case ElevatorState::MATCHLOAD:
         runMatchload();
         break;
      case ElevatorState::PICKUP:
         runPickup();
         break;
      case ElevatorState::MANUAL:
         runManual();
         break;
      }

      if (currentState == ElevatorState::SCORE && phase == 0 && getElevatorPosition() < 20.0)
        return;

      // ── Early exit if state changed during phase logic ──
      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
         return;

      // ── PID computation ──
      double elevPos = getElevatorPosition();
      double wristPos = getWristPosition();

      double elevErr = elevatorTarget - elevPos;
      double wristErr = wristTarget - wristPos;

      double elevOut = elevatorPID.update(elevErr);
      double wristOut = wristPID.update(wristErr);

      // ── Differential mixing ──
      // Left motor: -wrist + elevator
      // Right motor: -wrist - elevator
      leftPower = clamp127(-wristOut + elevOut);
      rightPower = clamp127(-wristOut - elevOut);

      // ── Zero motors when settled (prevents oscillation and heat) ──
      if (isSettled() && stateComplete)
      {
         leftPower = 0;
         rightPower = 0;
      }

      leftElevator.move(leftPower);
      rightElevator.move(rightPower);
   }
};

inline Elevator elevator;