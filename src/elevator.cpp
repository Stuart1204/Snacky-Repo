#ifndef ELEVATOR_CPP
#define ELEVATOR_CPP

#include "globals.hpp"
#include "piston.cpp"
#include "intake.cpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

/** Lift presets, motion tuning, and digital wrist timing for the linear elevator. */
namespace ElevatorConfig
{
   /** Piston values accepted directly by Piston::setState(). */
   constexpr bool WRIST_RETRACTED = false;
   constexpr bool WRIST_EXTENDED = true;

   constexpr double TICKS_PER_INCH = 83.0;

   /** Linear-travel limits and ordinary lift presets in inches. */
   constexpr double ELEVATOR_MIN = -100.0;
   constexpr double ELEVATOR_MAX = 100.0;
   constexpr double STOW_ELEV = 5.5;
   constexpr double SCORE_LIFT_DELTA_IN = 0.5;
   constexpr double RESET_ELEV = 7.0;       // Normal safe height for wrist extension.
   constexpr double FAST_RESET_ELEV = 3.0;  // Fast-reset safe height for wrist extension.
   constexpr double RESET_ELEV_SEEK = -100.0;
   constexpr double LOAD_ELEV = 0.0;
   constexpr double INTAKE_PICKUP_ELEV = 0.5;
   constexpr double MATCHLOAD_ELEV = 0.0;
   constexpr double MATCHLOAD_CLAMP_ELEV = -1.5;

   /** Stack-height presets in inches; PIN_HEIGHTS remains indexed from zero. */
   constexpr double PIN_0_ELEV = 0.5;
   constexpr double PIN_1_ELEV = 8.6;
   constexpr double PIN_2_ELEV = 15.6;
   constexpr double PIN_3_ELEV = 24.1;
   constexpr double PIN_4_ELEV = 32.6;
   constexpr double PIN_MAX_ELEV = 40.0;
   constexpr double PIN_HEIGHTS[] = {PIN_0_ELEV, PIN_1_ELEV, PIN_2_ELEV, PIN_3_ELEV, PIN_4_ELEV};
   constexpr int NUM_PIN_LEVELS = 5;
   constexpr double ALLIANCE_OFFSET = 3.5;

   /** Direct ground and match-load pickup heights in inches. */
   constexpr double GROUND_PICKUP_ELEV = 5.0;
   constexpr double GROUND_CLAMP_ELEV = -4.5;

   /** Linear-lift PID and completion tolerance. */
   constexpr double ELEV_KP = 45.0;
   constexpr double ELEV_KI = 0.0;
   constexpr double ELEV_KD = 0.0;
   constexpr double ELEV_SETTLE_IN = 1.0;

   /** Lift-stall detection used by reset and pickup motions. */
   constexpr double STALL_VELOCITY_THRESHOLD = 200.0;
   constexpr double STALL_CURRENT_THRESHOLD = 1200.0;
   constexpr uint32_t STALL_GRACE_PERIOD_MS = 150;
   constexpr uint32_t STALL_TIMEOUT_MS = 900;

   /** Global protection against sustained high-current lift stalls. */
   constexpr uint32_t GLOBAL_STALL_DURATION_MS = 500;
   constexpr double GLOBAL_STALL_VELOCITY = 20.0;
   constexpr double GLOBAL_STALL_CURRENT = 2000.0;

   /** Piston and object-detection timing/thresholds to tune on the new mechanism. */
   constexpr uint32_t WRIST_EXTEND_TIME_MS = 300;
   constexpr uint32_t WRIST_RETRACT_TIME_MS = 300;
   constexpr uint32_t CLAMP_ACTIVATE_DELAY_MS = 300;
   constexpr int CLAMP_DETECT_DISTANCE_MM = 100;
   constexpr int MATCHLOAD_DETECT_DISTANCE_MM = 140;
   constexpr int SCORE_RESET_TRAVEL_CENTIDEG = 2500;
}

/** State machine for a two-motor linear lift, digital wrist, and digital clamp. */
class Elevator
{
private:
   lemlib::PID elevatorPID{
       ElevatorConfig::ELEV_KP,
       ElevatorConfig::ELEV_KI,
       ElevatorConfig::ELEV_KD, 0};

   int currentState = ElevatorState::REST;
   int lastState = ElevatorState::REST;
   int phase = 0;
   bool stateComplete = false;

   double elevatorTarget = 0.0;
   double tempEleTarget = 0.0;
   double holdElevTarget = 0.0;
   double pickupElevTarget = 0.0;
   bool pickupUsesIntake = false;
   bool holdWristExtended = ElevatorConfig::WRIST_RETRACTED;
   bool holdWristFirst = false;

   int maxSpeed = 127;
   int currentPinLevel = 0;
   int odomPosition = 0;
   int leftPower = 0;
   int rightPower = 0;

   uint32_t motionStartTime = 0;
   uint32_t stallStartTime = 0;
   uint32_t wristLastChangeTime = 0;
   uint32_t globalStallStart = 0;
   bool globalStallActive = false;

   /** Clamps a numeric value to an inclusive range. */
   static double clampD(double value, double low, double high)
   {
      return (value < low) ? low : (value > high) ? high : value;
   }

   /** Returns true only for an installed sensor with a valid reading below the threshold. */
   static bool objectWithin(pros::Distance& sensor, int thresholdMM)
   {
      const int distanceMM = sensor.get_distance();
      return sensor.is_installed() && distanceMM >= 0 && distanceMM < thresholdMM;
   }

   /** Stops both intakevator motors and clears the values exposed by telemetry. */
   void stopMotors()
   {
      leftPower = 0;
      rightPower = 0;
      intakevator.move(0);
   }

   double getAverageVelocity()
   {
      return (fabs(intakevator.get_actual_velocity(0)) +
              fabs(intakevator.get_actual_velocity(1))) / 2.0;
   }

   double getAverageCurrent()
   {
      return (abs(intakevator.get_current_draw(0)) +
              abs(intakevator.get_current_draw(1))) / 2.0;
   }

   bool intakeOwnsMotors()
   {
      if (intake.getState() == IntakeState::IDLE) return false;
      if (intake.getState() == IntakeState::OUTTAKE) return true;
      return currentState == ElevatorState::REST || isElevatorSettled();
   }

   /** Commands the digital wrist only when its desired state actually changes. */
   void setWrist(bool extended)
   {
      if (wristPiston.getState() != extended)
      {
         wristPiston.setState(extended);
         wristLastChangeTime = pros::millis();
      }
   }

   /** Waits for the configured extension/retraction time because the piston has no analog feedback. */
   bool isWristReady() const
   {
      const uint32_t requiredTime = wristPiston.getState() ? ElevatorConfig::WRIST_EXTEND_TIME_MS
                                                            : ElevatorConfig::WRIST_RETRACT_TIME_MS;
      return pros::millis() - wristLastChangeTime >= requiredTime;
   }

   /** Reports whether both the lift target and wrist actuation delay are complete. */
   bool isSettled()
   {
      return isElevatorSettled() && isWristReady();
   }

   /** Detects a stalled lift while allowing for startup transients. */
   bool isMotionStalled()
   {
      if (pros::millis() - motionStartTime < ElevatorConfig::STALL_GRACE_PERIOD_MS)
      {
         stallStartTime = pros::millis();
         return false;
      }
      if (pros::millis() - motionStartTime > ElevatorConfig::STALL_TIMEOUT_MS)
         return true;

      const double avgVelocity = getAverageVelocity();
      const double avgCurrent = getAverageCurrent();
      const bool stalledNow = avgVelocity < ElevatorConfig::STALL_VELOCITY_THRESHOLD &&
                              avgCurrent > ElevatorConfig::STALL_CURRENT_THRESHOLD;

      if (!stalledNow)
         stallStartTime = pros::millis();
      return pros::millis() - stallStartTime > 90;
   }

   /** Stops the lift after a sustained high-current stall outside REST/ESTOP. */
   bool checkGlobalStallProtection()
   {
      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
         return false;

      const double avgVelocity = getAverageVelocity();
      const double avgCurrent = getAverageCurrent();
      const bool commandingHard = abs(leftPower) > 70 || abs(rightPower) > 70;
      const bool stalledNow = commandingHard &&
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
            return true;
         }
      }
      else
      {
         globalStallActive = false;
      }
      return false;
   }

   /** Starts timing and resets the linear-lift PID for a new motion. */
   void beginMotion()
   {
      motionStartTime = pros::millis();
      stallStartTime = pros::millis();
      elevatorPID.reset();
   }

   /** Holds either the wrist or elevator first, then completes the other action. */
   void runHold()
   {
      switch (phase)
      {
      case 0:
         if (holdWristFirst)
         {
            setWrist(holdWristExtended);
            if (isWristReady()) phase = 1;
         }
         else
         {
            elevatorTarget = holdElevTarget;
            if (isElevatorSettled()) phase = 1;
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
            setWrist(holdWristExtended);
            if (isWristReady())
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

   /** Stows the lift, then extends the wrist into its normal carrying position. */
   void runStow()
   {
      switch (phase)
      {
      case 0:
         setWrist(ElevatorConfig::WRIST_RETRACTED);
         elevatorTarget = ElevatorConfig::STOW_ELEV;
         if (isElevatorSettled()) phase = 1;
         break;
      case 1:
         setWrist(ElevatorConfig::WRIST_EXTENDED);
         if (isWristReady())
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

   /** Extends the wrist safely, drives down to the hard stop, and tares lift position. */
   void runReset()
   {
      switch (phase)
      {
      case 0:
      {
         const double safeExtensionHeight = isFastReset ? ElevatorConfig::FAST_RESET_ELEV
                                                        : ElevatorConfig::RESET_ELEV;
         if (wristPiston.getState() != ElevatorConfig::WRIST_EXTENDED &&
             getElevatorPosition() < safeExtensionHeight)
         {
            elevatorTarget = safeExtensionHeight;
            phase = 1;
         }
         else
         {
            setWrist(ElevatorConfig::WRIST_EXTENDED);
            phase = 2;
         }
         break;
      }
      case 1:
         if (isElevatorSettled())
         {
            setWrist(ElevatorConfig::WRIST_EXTENDED);
            phase = 2;
         }
         break;
      case 2:
         if (isWristReady())
         {
            elevatorTarget = ElevatorConfig::RESET_ELEV_SEEK;
            beginMotion();
            phase = 3;
         }
         break;
      case 3:
         if (isMotionStalled())
         {
            stopMotors();
            elevatorReset();
            stateComplete = true;
            setState(matchloadMode ? ElevatorState::MATCHLOAD : ElevatorState::LOAD);
         }
         break;
      }
   }


   /** Extends the wrist before lowering to the standing-ground-pin pickup height. */
   void runGroundPickup()
   {
      switch (phase)
      {
      case 0:
         setWrist(ElevatorConfig::WRIST_EXTENDED);
         if (isWristReady()) phase = 1;
         break;
      case 1:
         elevatorTarget = ElevatorConfig::GROUND_PICKUP_ELEV;
         if (isElevatorSettled() || isMotionStalled())
         {
            stateComplete = true;
            elevatorTarget = getElevatorPosition();
            phase = 2;
         }
         break;
      case 2:
         break;
      }
   }

   /** Prepares direct match-load capture without running the intake. */
   void runMatchload()
   {
      switch (phase)
      {
      case 0:
         setWrist(ElevatorConfig::WRIST_EXTENDED);
         elevatorTarget = ElevatorConfig::MATCHLOAD_ELEV;
         if (isSettled())
         {
            stateComplete = true;
            phase = 1;
         }
         break;
      case 1:
         if (objectWithin(matchloadSensor, ElevatorConfig::MATCHLOAD_DETECT_DISTANCE_MM))
         {
            setState(ElevatorState::PICKUP);
         }
         break;
      }
   }

   /** Moves to the pickup height and clamps only when the internal distance sensor sees a pin. */
   void runPickup()
   {
      switch (phase)
      {
      case 0:
         elevatorTarget = pickupElevTarget;
         if ((isElevatorSettled() || isMotionStalled()) &&
             objectWithin(clampDistSensor, ElevatorConfig::CLAMP_DETECT_DISTANCE_MM))
         {
            clamp.setState(true);
            if (pickupUsesIntake) intake.setState(IntakeState::IDLE);
            motionStartTime = pros::millis();
            phase = 1;
         }
         break;
      case 1:
         if (pros::millis() - motionStartTime >= ElevatorConfig::CLAMP_ACTIVATE_DELAY_MS)
         {
            setWrist(ElevatorConfig::WRIST_EXTENDED);
            phase = 2;
         }
         break;
      case 2:
         if (isWristReady()) goToPinHeight(1);
         break;
      }
   }

   /** Releases the clamp and lifts without changing wrist state. */
   void runScore()
   {
      elevatorTarget = tempEleTarget + ElevatorConfig::SCORE_LIFT_DELTA_IN;
      if (isElevatorSettled())
      {
         stateComplete = true;
         if (isAutoReset &&
             vertical_rotation.get_position() - odomPosition > ElevatorConfig::SCORE_RESET_TRAVEL_CENTIDEG)
         {
            setState(ElevatorState::RESET);
         }
      }
   }

   /** Cycles to the requested stack height while leaving the wrist in its current state. */
   void runPinHeight()
   {
      if (isElevatorSettled()) stateComplete = true;
   }

public:
   /** Configures lift brake behavior and a safe initial wrist state. */
   Elevator()
   {
      intakevator.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
      intakevatorPto.setState(false);
      wristPiston.setState(ElevatorConfig::WRIST_RETRACTED);
      wristLastChangeTime = pros::millis();
   }

   /**
    * Starts a named elevator state and initializes its hardware-specific actions.
    * @param state Value from ElevatorState such as LOAD, GROUND_PICKUP, or SCORE.
    */
   void setState(int state)
   {
      if (state == currentState) return;

      lastState = currentState;
      currentState = state;
      phase = 0;
      stateComplete = false;
      maxSpeed = 127;
      motionStartTime = pros::millis();
      stopMotors();
      beginMotion();

      switch (currentState)
      {
      case ElevatorState::STOW:
         currentPinLevel = 0;
         break;
      case ElevatorState::SCORE:
         odomPosition = vertical_rotation.get_position();
         tempEleTarget = getElevatorPosition();
         clamp.setState(false);
         break;
      case ElevatorState::RESET:
         break;
      case ElevatorState::LOAD:
         clamp.setState(false);
         hood.setState(false);
         if (isDropDownIntake) intakeLift.setState(false);
         if (lastState == ElevatorState::RESET) elevatorReset();
         break;
      case ElevatorState::MATCHLOAD:
         clamp.setState(false);
         hood.setState(true);
         intake.setState(IntakeState::IDLE);
         elevatorTarget = ElevatorConfig::MATCHLOAD_ELEV;
         setWrist(ElevatorConfig::WRIST_EXTENDED);
         break;
      case ElevatorState::PICKUP:
         clamp.setState(false);
         if (lastState == ElevatorState::GROUND_PICKUP || lastState == ElevatorState::GROUND_CLAMP)
         {
            pickupUsesIntake = false;
            pickupElevTarget = ElevatorConfig::GROUND_CLAMP_ELEV;
            setWrist(ElevatorConfig::WRIST_EXTENDED);
            intake.setState(IntakeState::IDLE);
         }
         else if (lastState == ElevatorState::MATCHLOAD || lastState == ElevatorState::MATCHLOAD_CLAMP)
         {
            pickupUsesIntake = false;
            pickupElevTarget = ElevatorConfig::MATCHLOAD_CLAMP_ELEV;
            setWrist(ElevatorConfig::WRIST_EXTENDED);
            intake.setState(IntakeState::IDLE);
         }
         else
         {
            pickupUsesIntake = true;
            pickupElevTarget = ElevatorConfig::INTAKE_PICKUP_ELEV;
            setWrist(ElevatorConfig::WRIST_RETRACTED);
            intake.setState(IntakeState::INTAKE);
         }
         break;
      case ElevatorState::GROUND_PICKUP:
         clamp.setState(false);
         intake.setState(IntakeState::IDLE);
         break;


      case ElevatorState::MANUAL:
         elevatorTarget = getElevatorPosition();
         break;
      case ElevatorState::PIN_HEIGHT:
      case ElevatorState::HOLD:
         break;
      case ElevatorState::REST:
      case ElevatorState::ESTOP:
         stopMotors();
         break;
      }
   }

   /** Limits the next lift output command to the PROS -127 to 127 range. */
   void setMaxSpeed(int speed) { maxSpeed = speed; }
   int getState() const { return currentState; }
   int getLastState() const { return lastState; }
   int getPhase() const { return phase; }
   bool isStateComplete() const { return stateComplete; }
   int getPinLevel() const { return currentPinLevel; }

   /** Returns a valid stack height with the neutral/alliance offset applied. */
   double getPinElevation(int level)
   {
      if (level < 0 || level >= ElevatorConfig::NUM_PIN_LEVELS)
      {
         return ElevatorConfig::PIN_1_ELEV -
                (isNeutralGoal ? 0.0 : ElevatorConfig::ALLIANCE_OFFSET);
      }
      return ElevatorConfig::PIN_HEIGHTS[level] -
             (isNeutralGoal ? 0.0 : ElevatorConfig::ALLIANCE_OFFSET);
   }

   /**
    * Moves to a selected stack level without introducing any wrist target.
    * @param level Index into PIN_HEIGHTS; values are clamped to the available levels.
    */
   void goToPinHeight(int level)
   {
      level = std::clamp(level, 0, ElevatorConfig::NUM_PIN_LEVELS - 1);
      currentPinLevel = level;
      elevatorTarget = clampD(getPinElevation(level), 0.0, ElevatorConfig::PIN_MAX_ELEV);

      if (currentState != ElevatorState::PIN_HEIGHT)
      {
         setState(ElevatorState::PIN_HEIGHT);
      }
      else
      {
         phase = 0;
         stateComplete = false;
         beginMotion();
      }
   }

   /** Selects the next defined stack level above the current measured lift position. */
   void goToNextPinAboveCurrent()
   {
      const double currentElevation = getElevatorPosition();
      for (int level = 1; level < ElevatorConfig::NUM_PIN_LEVELS; ++level)
      {
         if (getPinElevation(level) > currentElevation + 1.0)
         {
            goToPinHeight(level);
            return;
         }
      }
   }

   /**
    * Commands linear-lift travel while leaving the digital wrist unchanged.
    * @param direction Positive for upward travel, zero or negative for downward travel.
    */
   void manualElevator(int direction)
   {
      if (currentState != ElevatorState::MANUAL && currentState != ElevatorState::PIN_HEIGHT)
      {
         setState(ElevatorState::MANUAL);
      }
      elevatorTarget = direction > 0 ? ElevatorConfig::ELEVATOR_MAX : ElevatorConfig::ELEVATOR_MIN;
   }

   /** Stops manual motion and holds the current lift position. */
   void manualStop()
   {
      stopMotors();
      elevatorTarget = getElevatorPosition();
      elevatorPID.reset();
   }

   /** @return Averaged linear-lift travel in inches from the intakevator motor group. */
   double getElevatorPosition()
   {
      const double rightTicks = intakevator.get_position(0);
      const double leftTicks = intakevator.get_position(1);
      return (leftTicks + rightTicks) / 2.0 / ElevatorConfig::TICKS_PER_INCH;
   }

   /** @param inches New absolute lift target in inches. */
   void setElevatorTarget(double inches) { elevatorTarget = inches; }
   /** @return Current absolute lift target in inches. */
   double getElevatorTarget() const { return elevatorTarget; }
   /** @param delta Signed amount added to the lift target, in inches. */
   void changeElevatorTarget(double delta) { elevatorTarget += delta; }

   /** Reports whether the lift is inside its position tolerance. */
   bool isElevatorSettled()
   {
      return fabs(getElevatorPosition() - elevatorTarget) < ElevatorConfig::ELEV_SETTLE_IN;
   }

   /**
    * Holds a lift height and digital wrist state in the requested order.
    * @param elevInches Desired lift position in inches.
    * @param wristExtended True extends the wrist piston; false retracts it.
    * @param wristFirst True actuates the wrist before moving the lift; false moves the lift first.
    */
   void moveTo(double elevInches, bool wristExtended, bool wristFirst = false)
   {
      holdElevTarget = elevInches;
      holdWristExtended = wristExtended;
      holdWristFirst = wristFirst;
      setState(ElevatorState::HOLD);
   }

   /** Tares both intakevator motor encoders and resets the lift PID. */
   void elevatorReset()
   {
      intakevator.tare_position_all();
      elevatorTarget = 0.0;
      elevatorPID.reset();
   }

   /** Holds the current lift position without changing the wrist. */
   void holdCurrent()
   {
      elevatorTarget = getElevatorPosition();
      elevatorPID.reset();
   }

   /** Blocking helpers retained for autonomous sequencing. */
   void waitUntilSettled(uint32_t timeoutMs = 3000)
   {
      const uint32_t start = pros::millis();
      while (!isSettled() && pros::millis() - start < timeoutMs) pros::delay(10);
   }

   void waitUntilComplete(uint32_t timeoutMs = 5000)
   {
      const uint32_t start = pros::millis();
      while (!stateComplete && pros::millis() - start < timeoutMs) pros::delay(10);
   }

   void waitUntilPhase(int targetPhase, uint32_t timeoutMs = 3000)
   {
      const uint32_t start = pros::millis();
      while (phase < targetPhase && pros::millis() - start < timeoutMs) pros::delay(10);
   }

   void waitUntilElevatorSettled(uint32_t timeoutMs = 3000)
   {
      const uint32_t start = pros::millis();
      while (!isElevatorSettled() && pros::millis() - start < timeoutMs) pros::delay(10);
   }

   int getLeftPower() const { return leftPower; }
   int getRightPower() const { return rightPower; }

   /** Prints the linear lift and digital piston states without analog wrist values. */
   void printTelemetry()
   {
      printf("State:%d Phase:%d Pin:%d | Elev:%.1f->%.1f | Wrist:%d | L:%d R:%d\n",
             currentState, phase, currentPinLevel,
             getElevatorPosition(), elevatorTarget,
             wristPiston.getState(), leftPower, rightPower);
   }

   /** Runs lift logic and gives intake the intakevator only when lift motion is settled. */
   void run()
   {
      if (!intakeOwnsMotors() && checkGlobalStallProtection())
      {
         stopMotors();
         currentState = ElevatorState::ESTOP;
         return;
      }

      if (currentState == ElevatorState::REST || currentState == ElevatorState::ESTOP)
      {
         if (currentState == ElevatorState::REST && intake.getState() != IntakeState::IDLE)
         {
            intake.run();
         }
         else
         {
            stopMotors();
         }
         return;
      }

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
         elevatorTarget = ElevatorConfig::LOAD_ELEV;
         stateComplete = isElevatorSettled();
         break;
      case ElevatorState::MATCHLOAD:
         runMatchload();
         break;
      case ElevatorState::PICKUP:
         runPickup();
         break;
      case ElevatorState::MANUAL:
         break;
      case ElevatorState::HOLD:
         runHold();
         break;
      case ElevatorState::PIN_HEIGHT:
         runPinHeight();
         break;
      case ElevatorState::GROUND_PICKUP:
         clamp.setState(false);
         runGroundPickup();
         break;
      case ElevatorState::GROUND_CLAMP:
      case ElevatorState::MATCHLOAD_CLAMP:
         break;
      case ElevatorState::REST:
      case ElevatorState::ESTOP:
         return;
      }

      if (intakeOwnsMotors())
      {
         intake.run();
         return;
      }

      if (isSettled() && stateComplete)
      {
         stopMotors();
         return;
      }

      const double elevatorOutput = clampD(
          elevatorPID.update(elevatorTarget - getElevatorPosition()), -maxSpeed, maxSpeed);
      leftPower = static_cast<int>(elevatorOutput);
      rightPower = static_cast<int>(elevatorOutput);
      setIntakevatorPto(false);
      intakevator.move(elevatorOutput);
   }
};

/** Shared linear-elevator state machine used by controls and autonomous routines. */
inline Elevator elevator;

#endif // ELEVATOR_CPP
