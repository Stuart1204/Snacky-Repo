#ifndef other_functions_cpp
#define other_functions_cpp

#include "pros/optical.hpp"

#include <cmath>
#include <cstdint>

/** Colors that one optical sensor can report; UNKNOWN suppresses any downstream decisions. */
enum class VisibleColour : std::uint8_t {
   NONE,
   RED,
   BLUE,
   YELLOW,
   UNKNOWN
};

/** Raw optical measurements together with the color inferred from them. */
struct ColourSample {
   VisibleColour colour = VisibleColour::UNKNOWN; /**< Debounced input's inferred visible color. */
   double hue = 0.0; /**< PROS hue reading from 0 to 359.999 degrees. */
   double saturation = 0.0; /**< PROS saturation reading from 0 to 1. */
   double brightness = 0.0; /**< PROS brightness reading from 0 to 1. */
   std::int32_t proximity = 0; /**< PROS proximity reading from 0 to 255. */
};

/**
 * Samples one optical sensor and classifies the visible color.
 *
 * Proximity separates a nearby pin from the background. Saturation and brightness
 * reject unreliable color readings, and UNKNOWN is returned for disconnected or
 * ambiguous sensors. Thresholds are starting values that must be tuned on the robot.
 * @param opticalSensor Optical sensor to sample by reference.
 * @return Raw measurements and the inferred visible color.
 */
inline ColourSample checkColour(pros::Optical& opticalSensor) {
   constexpr std::int32_t PROXIMITY_THRESHOLD = 110; // Minimum 0-255 proximity for a nearby pin.
   constexpr double MIN_SATURATION = 0.25;          // Reject washed-out colors and background.
   constexpr double MIN_BRIGHTNESS = 0.02;          // Reject readings too dark for reliable hue.

   ColourSample sample;
   sample.hue = opticalSensor.get_hue();
   sample.saturation = opticalSensor.get_saturation();
   sample.brightness = opticalSensor.get_brightness();
   sample.proximity = opticalSensor.get_proximity();

   if (!opticalSensor.is_installed()) {
      sample.colour = VisibleColour::UNKNOWN;
   } else if (sample.proximity < PROXIMITY_THRESHOLD) {
      sample.colour = VisibleColour::NONE;
   } else if (!std::isfinite(sample.hue) || !std::isfinite(sample.saturation) ||
              !std::isfinite(sample.brightness) || sample.saturation < MIN_SATURATION ||
              sample.brightness < MIN_BRIGHTNESS) {
      sample.colour = VisibleColour::UNKNOWN;
   } else if (sample.hue < 30.0 || sample.hue > 330.0) {
      // Red wraps around the 0/360-degree hue boundary.
      sample.colour = VisibleColour::RED;
   } else if (sample.hue >= 40.0 && sample.hue <= 90.0) {
      sample.colour = VisibleColour::YELLOW;
   } else if (sample.hue >= 180.0 && sample.hue <= 270.0) {
      sample.colour = VisibleColour::BLUE;
   } else {
      sample.colour = VisibleColour::UNKNOWN;
   }
   return sample;
}

#endif
