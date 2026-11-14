#pragma once

#include "globals.hpp"
#include "other_functions.cpp"
#include "pros/rtos.hpp"

#include <cstdint>

/** Color combinations detected across the two halves of an Override pin. */
enum class PinType : std::uint8_t {
   NONE,
   RED_BLUE,
   RED_YELLOW,
   BLUE_YELLOW,
   YELLOW_YELLOW,
   UNKNOWN
};

/** Synchronized upper/lower classifications and the raw samples that produced them. */
struct PinReading {
   PinType type = PinType::UNKNOWN;
   ColourSample upper;
   ColourSample lower;
};

/**
 * Returns a short display label for a detected pin pair.
 * @param type Detected combination of the two pin halves.
 * @return Constant string suitable for controller or brain-screen display.
 */
inline const char* pinTypeName(PinType type) {
   switch (type) {
      case PinType::NONE: return "NONE";
      case PinType::RED_BLUE: return "R/B";
      case PinType::RED_YELLOW: return "R/Y";
      case PinType::BLUE_YELLOW: return "B/Y";
      case PinType::YELLOW_YELLOW: return "Y/Y";
      case PinType::UNKNOWN: return "UNKNOWN";
   }
   return "UNKNOWN";
}

/**
 * Read-only dual-optical pin classifier.
 *
 * Each half requires three identical consecutive samples before its stable color
 * changes. The class only reports readings; it has no intake, elevator, or sorting
 * output. A mutex keeps the 20 ms sampling task and display task consistent.
 */
class PinColourDetector {
private:
   /** Candidate/stable color state for one optical sensor. */
   struct DebouncedColour {
      VisibleColour candidate = VisibleColour::UNKNOWN;
      VisibleColour stable = VisibleColour::UNKNOWN;
      std::uint8_t count = 0;
   };

   pros::Optical& upperSensor;
   pros::Optical& lowerSensor;
   DebouncedColour upperState;
   DebouncedColour lowerState;
   PinReading latest;
   pros::Mutex mutex;

   /** Samples one half and updates its three-sample debounce state. */
   static VisibleColour updateSide(pros::Optical& sensor, DebouncedColour& state, ColourSample& raw) {
      raw = checkColour(sensor);
      if (raw.colour == state.candidate) {
         if (state.count < 255) ++state.count;
      } else {
         state.candidate = raw.colour;
         state.count = 1;
      }
      if (state.count >= 3) state.stable = state.candidate;
      return state.stable;
   }

   /** Checks a two-color pair without depending on upper/lower orientation. */
   static bool isPair(VisibleColour first, VisibleColour second, VisibleColour a, VisibleColour b) {
      return (first == a && second == b) || (first == b && second == a);
   }

   /** Combines two stable half colors; missing or ambiguous halves produce UNKNOWN. */
   static PinType classify(VisibleColour upper, VisibleColour lower) {
      if (upper == VisibleColour::NONE && lower == VisibleColour::NONE) return PinType::NONE;
      if (upper == VisibleColour::UNKNOWN || lower == VisibleColour::UNKNOWN ||
          upper == VisibleColour::NONE || lower == VisibleColour::NONE) {
         return PinType::UNKNOWN;
      }
      if (isPair(upper, lower, VisibleColour::RED, VisibleColour::BLUE)) return PinType::RED_BLUE;
      if (isPair(upper, lower, VisibleColour::RED, VisibleColour::YELLOW)) return PinType::RED_YELLOW;
      if (isPair(upper, lower, VisibleColour::BLUE, VisibleColour::YELLOW)) return PinType::BLUE_YELLOW;
      if (upper == VisibleColour::YELLOW && lower == VisibleColour::YELLOW) return PinType::YELLOW_YELLOW;
      return PinType::UNKNOWN;
   }

public:
   /**
    * References the optical sensors aimed at the upper and lower pin halves.
    * @param upperSensor Optical sensor viewing the upper pin half.
    * @param lowerSensor Optical sensor viewing the lower pin half.
    */
   PinColourDetector(pros::Optical& upperSensor, pros::Optical& lowerSensor)
       : upperSensor(upperSensor), lowerSensor(lowerSensor) {}

   /** Configures both sensors for fast, consistently illuminated color sampling. */
   void initialize() {
      for (pros::Optical* sensor : {&upperSensor, &lowerSensor}) {
         sensor->disable_gesture();
         sensor->set_led_pwm(100);
         sensor->set_integration_time(10.0);
      }
   }

   /** @return Atomic snapshot containing both raw samples and the classified pin pair. */
   PinReading update() {
      mutex.lock();
      const VisibleColour upper = updateSide(upperSensor, upperState, latest.upper);
      const VisibleColour lower = updateSide(lowerSensor, lowerState, latest.lower);
      latest.type = classify(upper, lower);
      const PinReading reading = latest;
      mutex.unlock();
      return reading;
   }

   /** @return Last completed classification snapshot without commanding any mechanism. */
   PinReading getTelemetry() {
      mutex.lock();
      const PinReading reading = latest;
      mutex.unlock();
      return reading;
   }
};

/** Shared detector configured by the two optical port objects in globals.hpp. */
inline PinColourDetector pinColourDetector(upperPinColourSensor, lowerPinColourSensor);
