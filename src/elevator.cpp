#pragma once

#include "globals.hpp"
#include "main.h"
#include "piston.cpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "intake.cpp"
#include "pros/rtos.hpp"

namespace ElevatorConfig
{
   constexpr double TICKS_PER_INCH = 83;
   constexpr double SENSOR_DEG_PER_WRIST_DEG = 1.0;

   constexpr double ELEVATOR_MIN = -200.0;
   constexpr double ELEVATOR_MAX = 100.0;
   constexpr double WRIST_MIN = -300.0;
   constexpr double WRIST_MAX = 90.0;

   constexpr double STOW_ELEV = 6.0;
   constexpr double STOW_WRIST = -75.0;

   constexpr double SCORE_ELEV = -10.0;
   constexpr double SCORE_WRIST = -65.0;

   constexpr double RESET_ELEV = 10;
   constexpr double RESET_WRIST = -170.0;
   constexpr double RESET_ELEV_SEEK = -100.0;

   constexpr double LOAD_ELEV = 0;
   constexpr double LOAD_WRIST = -180.0;

   constexpr double MATCHLOAD_ELEV = 0;
   constexpr double MATCHLOAD_WRIST = -175.0;

   constexpr double PICKUP_APPROACH_ELEV = 10.0;
   constexpr double PICKUP_VERTICAL_WRIST = -170.0;
   constexpr double PICKUP_SEEK_ELEV = 2;
   constexpr double PICKUP_SWEEP_WRIST = -170.0;

   constexpr double PIN_1_ELEV = 9.0;
   constexpr double PIN_2_ELEV = 15.0;
   constexpr double PIN_3_ELEV = 21.0;
   constexpr double PIN_4_ELEV = 27.0;
   constexpr double PIN_MAX_ELEV = 28.0; // clamp guard
   constexpr double PIN_WRIST = -90.0;

   constexpr double MANUAL_WRIST = STOW_WRIST;

   constexpr double ELEV_KP = 45.0, ELEV_KI = 0.0, ELEV_KD = 0.0;
   constexpr double WRIST_KP = 2.5, WRIST_KI = 0.05, WRIST_KD = 2;

   constexpr double ELEV_SETTLE_IN = 1;
   constexpr double WRIST_SETTLE_DEG = 8.0;

   constexpr double STALL_VELOCITY_THRESHOLD = 90.0;
   constexpr double STALL_CURRENT_THRESHOLD = 1600.0;
   constexpr uint32_t STALL_GRACE_PERIOD_MS = 300;
   constexpr uint32_t STALL_TIMEOUT_MS = 1000;

   constexpr uint32_t GLOBAL_STALL_DURATION_MS = 500;
   constexpr double GLOBAL_STALL_VELOCITY = 20.0;
   constexpr double GLOBAL_STALL_CURRENT = 2000.0;

   constexpr uint32_t CLAMP_ACTIVATE_DELAY_MS = 300;

   constexpr int MANUAL_VOLTAGE = 120;

   // All pin heights in order for cycling
   constexpr double PIN_HEIGHTS[] = {PIN_1_ELEV, PIN_2_ELEV, PIN_3_ELEV, PIN_4_ELEV};
   constexpr int NUM_PIN_LEVELS = 4;
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
   int phase = 0;
   bool stateComplete = false;

   int pendingNextState = ElevatorState::REST;
   int pendingIntakeState = IntakeState::IDLE;
   bool resetSequenceRequested = false;

   double elevatorTarget = 0.0;
   double wristTarget = 0.0;

   int maxSpeed = 127;

   double holdElevTarget = 0.0;
   double holdWristTarget = 0.0;
   bool holdWristFirst = false;

   int currentPinLevel = 0;

   int leftPower = 0;
   int rightPower = 0;

   uint32_t motionStartTime = 0;
   uint32_t stallStartTime = 0;
   uint32_t settleStartTime = 0;

   uint32_t globalStallStart = 0;
   bool globalStallActive = false;

   bool useSensorPickup = false;
   std::function<bool()> sensorDetectFunc = nullptr;

   static double clampD(double v, double lo, double hi)
   {
      return (v < lo) ? lo : (v > hi) ? hi : v;
   }

   static double wrapAngleDeg(double angle)
   {
      while (angle > 180.0)
         angle -= 360.0;
      while (angle <= -180.0)
         angle += 360.0;
      return angle;
   }

   static double angleErrorDeg(double target, double current)
   {
      return wrapAngleDeg(target - current);
   }

   // Check if wrist is "roughly vertical" (within 20 degrees of RESET_WRIST)
   bool isWristNearVertical()
   {
      return fabs(angleErrorDeg(ElevatorConfig::RESET_WRIST, getWristPosition())) < 10.0;
   }

   bool isMotionStalled()
   {
      if (pros::millis() - motionStartTime < ElevatorConfig::STALL_GRACE_PERIOD_MS)
      {
         stallStartTime = pros::millis();
         return false;
      }
      if (pros::millis() - motionStartTime > ElevatorConfig::STALL_TIMEOUT_MS)
         return true;

      double avgVelocity = (fabs(leftElevator.get_actual_velocity()) +
                            fabs(rightElevator.get_actual_velocity())) / 2.0;
      double avgCurrent = (fabs(leftElevator.get_current_draw()) +
                           fabs(rightElevator.get_current_draw())) / 2.0;

      bool stalledNow = (avgVelocity < ElevatorConfig::STALL_VELOCITY_THRESHOLD &&
                         avgCurrent > ElevatorConfig::STALL_CURRENT_THRESHOLD);

      if (!stalledNow)
         stallStartTime = pros::millis();

      return (pros::millis() - stallStartTime > 100);
   }

   bool checkGlobalStallProtection()
   {
      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
         return false;

      double avgVelocity = (fabs(leftElevator.get_actual_velocity()) +
                            fabs(rightElevator.get_actual_velocity())) / 2.0;
      double avgCurrent = (fabs(leftElevator.get_current_draw()) +
                           fabs(rightElevator.get_current_draw())) / 2.0;
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
            return true;
      }
      else
         globalStallActive = false;

      return false;
   }

   void beginMotion()
   {
      motionStartTime = pros::millis();
      stallStartTime = pros::millis();
      elevatorPID.reset();
      wristPID.reset();
   }

   // ═══════════════════════════════════════════════════════
   //  PHASE RUNNERS
   // ═══════════════════════════════════════════════════════

   void runHold()
   {
      switch (phase)
      {
      case 0:
         if (holdWristFirst)
         {
            wristTarget = holdWristTarget;
            elevatorTarget = getElevatorPosition();
            if (isWristSettled())
               phase = 1;
         }
         else
         {
            elevatorTarget = holdElevTarget;
            wristTarget = getWristPosition();
            if (isElevatorSettled())
               phase = 1;
         }
         break;
      case 1:
         if (holdWristFirst)
         {
            elevatorTarget = holdElevTarget;
            if (isElevatorSettled())
            {
               phase = 2;
               stateComplete = true;
            }
         }
         else
         {
            wristTarget = holdWristTarget;
            if (isWristSettled())
            {
               phase = 2;
               stateComplete = true;
            }
         }
         break;
      case 2:
         break;
      }
   }

   void runStow()
   {
      switch (phase)
      {
      case 0:
         elevatorTarget = ElevatorConfig::STOW_ELEV;
         wristTarget = -180.0;
         if (isElevatorSettled())
            phase = 1;
         break;
      case 1:
         wristTarget = ElevatorConfig::STOW_WRIST;
         if (isWristSettled())
         {
            phase = 2;
            stateComplete = true;
            elevatorTarget = getElevatorPosition();
         }
         break;
      case 2:
         break;
      }
   }

   void runReset()
   {
      // Phase 0: Decide path based on wrist and elevator position
      // Phase 1: Rotate wrist vertical (only if needed and we raised first)
      // Phase 2: Drive down until stalled on hard stop
      // Phase 3: Reset encoders, transition
      switch (phase)
      {
      case 0:
         if (isWristNearVertical())
         {
            // Wrist already vertical — just go straight down
            elevatorTarget = ElevatorConfig::RESET_ELEV_SEEK;
            beginMotion();
            phase = 2;
         }
         else if (getElevatorPosition() >= 10.0)
         {
            // High enough — rotate wrist in place, skip raising
            wristTarget = ElevatorConfig::RESET_WRIST;
            elevatorTarget = getElevatorPosition(); // hold elevator
            phase = 1;
         }
         else
         {
            // Too low and wrist not vertical — raise first
            elevatorTarget = ElevatorConfig::RESET_ELEV;
            wristTarget = getWristPosition(); // freeze wrist while raising
            phase = 4; // go to raise-then-rotate path
         }
         break;

      case 4:
         // Sub-path: raising elevator before rotating wrist
         elevatorTarget = ElevatorConfig::RESET_ELEV;
         wristTarget = getWristPosition();
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

         if (resetSequenceRequested)
         {
            int nextState = pendingNextState;
            int intakeMode = pendingIntakeState;
            pendingNextState = ElevatorState::REST;
            pendingIntakeState = IntakeState::IDLE;
            resetSequenceRequested = false;

            if (intakeMode != IntakeState::IDLE)
               intake.setState(intakeMode);

            setState(nextState);
         }
         break;
      }
   }

   void runLoad()
   {
      elevatorTarget = ElevatorConfig::LOAD_ELEV;
      wristTarget = ElevatorConfig::LOAD_WRIST;
      if (isSettled())
         stateComplete = true;
   }

   void runMatchload()
   {
      switch (phase)
      {
      case 0:
         if(pros::millis() - motionStartTime > 800) {
            phase = 1;
         }
         break;
      case 1:
         if (matchloadSensor.get_distance() < 100)
         {
            hood.setState(false);
            motionStartTime = pros::millis();
            phase = 2;
         }
         break;
      case 2:
         if (pros::millis() - motionStartTime > 500) {
            hood.setState(true);
            setState(ElevatorState::PICKUP);
         }
         break;
      }
   }

   void runPickup()
   {
      // Phase 0: Lower elevator to grab
      // Phase 1: Rotate wrist down
      // Phase 2: Clamp and wait
      // Phase 3: Elevator to pin 1 height
      // Phase 4: Wrist to pin angle → complete
      switch (phase)
      {
      case 0:
         intake.setState(IntakeState::INTAKE);
         elevatorTarget = 0.5;
         if (isElevatorSettled() || isMotionStalled())
            phase = 1;
         break;
      case 1:
         wristTarget = -180;
         if (isWristSettled() || isMotionStalled())
            phase = 2;
         break;
      case 2:
         clamp.setState(true);
         intake.setState(IntakeState::IDLE);
         if (pros::millis() - motionStartTime > 300)
            phase = 3;
         break;
      case 3:
         elevatorTarget = clampD(ElevatorConfig::PIN_1_ELEV,
                                 0.0, ElevatorConfig::PIN_MAX_ELEV);
         if (isElevatorSettled())
            phase = 4;
         break;
      case 4:
         wristTarget = ElevatorConfig::PIN_WRIST;
         if (isWristSettled())
         {
            currentPinLevel = 1;
            stateComplete = true;
            // Transition to PIN_HEIGHT so B button cycling works
            lastState = currentState;
            currentState = ElevatorState::PIN_HEIGHT;
            phase = 0;
            // Don't call setState — avoid re-init, just swap state ID
         }
         break;
      }
   }

   void runPinHeight()
   {
      // Elevator and wrist targets are already set by goToPinHeight()
      // Phase 0: move elevator
      // Phase 1: move wrist
      // Phase 2: done
      switch (phase)
      {
      case 0:
         if (isElevatorSettled())
            phase = 1;
         break;
      case 1:
         wristTarget = ElevatorConfig::PIN_WRIST;
         if (isWristSettled())
         {
            phase = 2;
            stateComplete = true;
         }
         break;
      case 2:
         break;
      }
   }

   void runManual()
   {
      // Nothing — PID is bypassed in run() when manual override is active.
      // When no override is active, PID holds current position.
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

      if (state != ElevatorState::RESET)
      {
         resetSequenceRequested = false;
         pendingNextState = ElevatorState::REST;
         pendingIntakeState = IntakeState::IDLE;
      }

      lastState = currentState;
      currentState = state;
      phase = 0;
      stateComplete = false;
      maxSpeed = 127;
      motionStartTime = pros::millis();

      leftElevator.move(0);
      rightElevator.move(0);

      beginMotion();

      switch (currentState)
      {
      case ElevatorState::STOW:
         currentPinLevel = 0;
         break;
      case ElevatorState::SCORE:
         clamp.setState(false);
         elevatorTarget = getElevatorPosition();
         wristTarget = ElevatorConfig::SCORE_WRIST;
         break;
      case ElevatorState::RESET:
         break;
      case ElevatorState::LOAD:
         clamp.setState(false);
         hood.setState(false);
         intakeLift.setState(false);
         break;
      case ElevatorState::MATCHLOAD:
         clamp.setState(false);
         hood.setState(true);
         intakeLift.setState(true);
         elevatorTarget = ElevatorConfig::MATCHLOAD_ELEV;
         wristTarget = ElevatorConfig::MATCHLOAD_WRIST;
         break;
      case ElevatorState::PICKUP:
         motionStartTime = pros::millis();
         break;
      case ElevatorState::MANUAL:
         elevatorTarget = getElevatorPosition();
         wristTarget = getWristPosition();
         break;
      case ElevatorState::PIN_HEIGHT:
         break;
      case ElevatorState::REST:
      case ElevatorState::ESTOP:
         leftElevator.move(0);
         rightElevator.move(0);
         break;
      case ElevatorState::HOLD:
         break;
      }
   }

   void setMaxSpeed(int speed) { maxSpeed = speed; }
   int getState() const { return currentState; }
   int getLastState() const { return lastState; }
   int getPhase() const { return phase; }
   bool isComplete() const { return stateComplete; }
   bool isStateComplete() const { return stateComplete; }
   int getPinLevel() const { return currentPinLevel; }

   // ═══════════════════════════════════════════════════════
   //  PIN HEIGHT
   // ═══════════════════════════════════════════════════════

   double getPinElevation(int level)
   {
      if (level < 1 || level > ElevatorConfig::NUM_PIN_LEVELS)
         return ElevatorConfig::PIN_1_ELEV;
      return ElevatorConfig::PIN_HEIGHTS[level - 1];
   }


   void forceResetAndIntake(int intakeMode, int nextState)
   {
      pendingNextState = nextState;
      pendingIntakeState = intakeMode;
      resetSequenceRequested = true;

      // Force re-entry even if already in RESET
      currentState = -1;
      setState(ElevatorState::RESET);
   }
   bool isAtResetHome(double elevTol = 3.0, double wristTol = 15.0)
   {
      return fabs(getElevatorPosition()) < elevTol &&
            fabs(getWristPosition() - (-180.0)) < wristTol;
   }

   void forceResetThen(int nextState, int intakeMode = IntakeState::IDLE)
   {
      pendingNextState = nextState;
      pendingIntakeState = intakeMode;
      resetSequenceRequested = true;
      currentState = -1;
      setState(ElevatorState::RESET);
   }

   void goToPinHeight(int level)
   {
      if (level < 1) level = 1;
      if (level > ElevatorConfig::NUM_PIN_LEVELS) level = ElevatorConfig::NUM_PIN_LEVELS;

      currentPinLevel = level;
      elevatorTarget = clampD(getPinElevation(level), 0.0, ElevatorConfig::PIN_MAX_ELEV);
      wristTarget = ElevatorConfig::PIN_WRIST;

      if (currentState != ElevatorState::PIN_HEIGHT)
      {
         currentState = -1;
         setState(ElevatorState::PIN_HEIGHT);
      }
      else
      {
         phase = 0;
         stateComplete = false;
         beginMotion();
      }
   }

   // Find the next pin level higher than current elevator position
   void goToNextPinAboveCurrent()
   {
      double curElev = getElevatorPosition();

      for (int i = 0; i < ElevatorConfig::NUM_PIN_LEVELS; i++)
      {
         if (ElevatorConfig::PIN_HEIGHTS[i] > curElev + 1.0)
         {
            goToPinHeight(i + 1);
            return;
         }
      }
      // Already at or above pin 4 — do nothing
   }

   // ═══════════════════════════════════════════════════════
   //  MANUAL MOTOR CONTROL
   //  Bypasses PID entirely. Applied directly in run().
   // ═══════════════════════════════════════════════════════

   // Call each cycle from controls to apply raw voltage.
   // Positive elevator = up, positive wrist = one direction.
   void manualElevator(int voltage)
   {
      if (currentState != ElevatorState::MANUAL &&
          currentState != ElevatorState::PIN_HEIGHT)
      {
         setState(ElevatorState::MANUAL);
      }

      // Differential: left = -wrist + elev, right = -wrist - elev
      // For pure elevator: left = +voltage, right = -voltage
      leftPower = clampD(voltage, -127, 127);
      rightPower = clampD(-voltage, -127, 127);

      leftElevator.move(leftPower);
      rightElevator.move(rightPower);

      // Track position so PID doesn't jerk when manual stops
      elevatorTarget = getElevatorPosition();
      wristTarget = getWristPosition();
   }

   void manualWrist(int voltage)
   {
      if (currentState != ElevatorState::MANUAL &&
          currentState != ElevatorState::PIN_HEIGHT)
      {
         setState(ElevatorState::MANUAL);
      }

      // For pure wrist: left = -voltage, right = -voltage (both same dir)
      leftPower = clampD(-voltage, -127, 127);
      rightPower = clampD(-voltage, -127, 127);

      leftElevator.move(leftPower);
      rightElevator.move(rightPower);

      elevatorTarget = getElevatorPosition();
      wristTarget = getWristPosition();
   }

   // Flag set by controls to tell run() to skip PID this cycle
   bool manualActiveThisCycle = false;

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
      if (pos > 62.5)
         pos -= 360.0;
      return pos;
   }

   void setElevatorTarget(double inches) { elevatorTarget = inches; }

   void setWristTarget(double degrees)
   {
      wristTarget = clampD(degrees, ElevatorConfig::WRIST_MIN, ElevatorConfig::WRIST_MAX);
   }

   double getElevatorTarget() const { return elevatorTarget; }
   double getWristTarget() const { return wristTarget; }

   void changeElevatorTarget(double delta) { setElevatorTarget(elevatorTarget + delta); }
   void changeWristTarget(double delta) { setWristTarget(wristTarget + delta); }

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

   // bool hasGamePiece()
   // {
   //    return colourSortSensor.get_proximity() > 100;
   // }

   void moveTo(double elevInches, double wristDeg, bool wristFirst = false)
   {
      holdElevTarget = elevInches;
      holdWristTarget = wristDeg;
      holdWristFirst = wristFirst;
      currentState = -1;
      setState(ElevatorState::HOLD);
   }

   void requestResetAndIntake(int intakeMode, int nextState)
   {
      if (currentState == ElevatorState::RESET && !stateComplete)
         return;

      pendingNextState = nextState;
      pendingIntakeState = intakeMode;
      resetSequenceRequested = true;
      setState(ElevatorState::RESET);
   }

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

   void waitUntilElevatorSettled(uint32_t timeoutMs = 3000)
   {
      uint32_t start = pros::millis();
      while (!isElevatorSettled() && (pros::millis() - start) < timeoutMs)
         pros::delay(10);
   }

   void waitUntilWristSettled(uint32_t timeoutMs = 3000)
   {
      uint32_t start = pros::millis();
      while (!isWristSettled() && (pros::millis() - start) < timeoutMs)
         pros::delay(10);
   }

   int getLeftPower() const { return leftPower; }
   int getRightPower() const { return rightPower; }

   void printTelemetry()
   {
      printf("State:%d Phase:%d Pin:%d | Elev:%.1f->%.1f | Wrist:%.1f->%.1f | L:%d R:%d\n",
             currentState, phase, currentPinLevel,
             getElevatorPosition(), elevatorTarget,
             getWristPosition(), wristTarget,
             leftPower, rightPower);
   }

   void run()
   {
      if (checkGlobalStallProtection())
      {
         leftElevator.move(0);
         rightElevator.move(0);
         leftPower = 0;
         rightPower = 0;
         currentState = ElevatorState::ESTOP;
         return;
      }

      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
      {
         leftElevator.move(0);
         rightElevator.move(0);
         leftPower = 0;
         rightPower = 0;
         return;
      }

      // ── If manual control was applied this cycle, skip PID entirely ──
      if (manualActiveThisCycle)
      {
         manualActiveThisCycle = false;
         return; // motors already set by manualElevator/manualWrist
      }

      switch (currentState)
      {
      case ElevatorState::STOW:
         runStow();
         break;
      case ElevatorState::SCORE:
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
      case ElevatorState::HOLD:
         runHold();
         break;
      case ElevatorState::PIN_HEIGHT:
         runPinHeight();
         break;
      }

      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
         return;

      double elevPos = getElevatorPosition();
      double wristPos = getWristPosition();

      double elevErr = elevatorTarget - elevPos;
      double wristErr = wristTarget - wristPos;

      double elevOut = elevatorPID.update(elevErr);
      double wristOut = wristPID.update(wristErr);

      leftPower = -wristOut + elevOut;
      rightPower = -wristOut - elevOut;

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