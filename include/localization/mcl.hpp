#pragma once

#include "pros/distance.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <random>
#include <vector>

/**
 * Inch/degree Monte Carlo localization primitives adapted from Echo.
 *
 * Poses use a standard mathematical frame: +X is zero degrees and angles increase
 * counterclockwise. Radians are used only inside trigonometric calculations. This
 * module has no motion-profile, motor-output, or LemLib odometry-task dependency.
 */
namespace mcl {

/** Shared math and process-model constants; tune noise on the physical robot. */
constexpr float PI = 3.14159265358979323846f;
constexpr float DEG_TO_RAD = PI / 180.0f;
constexpr float DRIVE_NOISE_FRACTION = 0.02f;       // Reduced drive diffusion; sensors dominate corrections.
constexpr float ANGLE_NOISE_DEG = 0.25f;            // Strong IMU trust: +/-0.25 degrees per update.
constexpr float MAX_DISTANCE_SINCE_UPDATE_IN = 0.5f; // Resample after only 0.5 inch of travel.
constexpr std::uint32_t MAX_UPDATE_INTERVAL_MS = 200;  // Correct at least every 100 ms while running.

/** Two-dimensional position or displacement in inches. */
struct Vec2 {
    float x = 0.0f;
    float y = 0.0f;
};

/** Estimated field position in inches and standard CCW heading in degrees. */
struct Pose {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

/** Mechanical and measurement configuration for one wall-facing distance sensor. */
struct DistanceConfig {
    pros::Distance* device = nullptr; /**< PROS device read by this adapter. */
    float forwardIn = 0.0f; /**< Sensor position forward of the tracking center, in inches. */
    float leftIn = 0.0f; /**< Sensor position left of the tracking center, in inches. */
    float heightIn = 0.0f; /**< Beam height in inches; selects which field features can intersect it. */
    float yawDeg = 0.0f; /**< Beam direction relative to the robot, CCW-positive in degrees. */
    float scale = 1.0f; /**< Calibration multiplier applied after millimetre-to-inch conversion. */
    float stdDevIn = 0.0f; /**< Optional minimum standard deviation; zero uses only the distance/confidence model. */
    bool enabled = true; /**< Excludes the sensor without changing its wiring or declaration. */
};

/** Nearest expected ray distance and whether this ray may affect particle weights. */
struct ExpectedDistance {
    float distanceIn = 0.0f;
    bool usable = false;
};

/** Interface implemented by a game-specific field map. */
class FieldModel {
public:
    /**
     * Predicts what a distance sensor should see for a candidate particle.
     * @param particle Candidate field pose in inches/degrees.
     * @param sensor Mechanical transform and calibration for the queried sensor.
     * @return Expected ray distance in inches and whether the ray is usable.
     */
    virtual ExpectedDistance expectedDistance(const Pose& particle, const DistanceConfig& sensor) const = 0;
    /** @return The field's center-to-wall distance in inches. */
    virtual float halfFieldExtent() const = 0;
    virtual ~FieldModel() = default;
};

/**
 * Returns the shortest signed angular error, wrapped to +/-180 degrees.
 * @param targetDeg Desired heading in degrees.
 * @param currentDeg Current heading in degrees.
 * @return Signed target-minus-current angle in degrees.
 */
inline float angleDifference(float targetDeg, float currentDeg) {
    return std::remainder(targetDeg - currentDeg, 360.0f);
}

/**
 * Rotates a vector in the standard CCW frame.
 * @param vector Input x/y components in inches.
 * @param angleDeg CCW rotation in degrees.
 * @return Rotated vector in inches.
 */
inline Vec2 rotate(Vec2 vector, float angleDeg) {
    const float angle = angleDeg * DEG_TO_RAD;
    const float cosine = std::cos(angle);
    const float sine = std::sin(angle);
    return {vector.x * cosine - vector.y * sine, vector.x * sine + vector.y * cosine};
}

/** Converts PROS distance readings into high-confidence particle likelihoods. */
class DistanceSensor {
private:
    DistanceConfig config;
    float measuredIn = 0.0f;
    float stdDevIn = 0.0f;
    bool valid = false;

public:
    /**
     * Stores one sensor's mechanical configuration and starts with no valid measurement.
     * @param config Device pointer, physical transform, scale, and uncertainty settings.
     */
    explicit DistanceSensor(DistanceConfig config) : config(config), stdDevIn(config.stdDevIn) {}

    /** Reads and validates the latest measurement, converting millimetres to inches once. */
    void update() {
        valid = false;
        if (!config.enabled || config.device == nullptr || !config.device->is_installed()) return;

        const std::int32_t measuredMM = config.device->get();
        const std::int32_t confidence = config.device->get_confidence();
        if (measuredMM < 0 || measuredMM >= 9999 || confidence <= 0) return;

        measuredIn = static_cast<float>(measuredMM) * 0.03937007874f * config.scale;
        // Strong sensor trust: 5% distance noise scaled by confidence/64.
        const float confidenceScale = std::sqrt(64.0f / static_cast<float>(confidence));
        stdDevIn = std::max(config.stdDevIn, 0.05f * measuredIn * confidenceScale);
        valid = std::isfinite(measuredIn) && std::isfinite(stdDevIn) && stdDevIn > 0.0f;
    }

    /**
     * Scores agreement between the measured ray and a particle's predicted ray.
     * @param particle Candidate field pose in inches/degrees.
     * @param field Field map used to predict the sensor distance.
     * @return Nonnegative likelihood; larger values indicate better agreement.
     */
    double likelihood(const Pose& particle, const FieldModel& field) const {
        if (!valid) return 1.0;

        const ExpectedDistance expected = field.expectedDistance(particle, config);
        if (!expected.usable) return 1.0;

        const float residual = measuredIn - expected.distanceIn;
        const float normalized = std::clamp(residual / stdDevIn, -50.0f, 50.0f);
        const float squared = normalized * normalized;
        const double weight = 0.3989422804014337 / (1.0 + 0.59422804014337 * squared * squared);
        return std::max(weight, 0.000000000001);
    }

    /** @return True when the latest reading passed installation and validity checks. */
    bool isValid() const { return valid; }
};

/**
 * Particle filter driven by tracking-wheel deltas and distance-sensor likelihoods.
 *
 * The filter owns its particles and random engine. It never writes to LemLib; the
 * adapter decides whether a fresh estimate is safe to apply to the chassis.
 */
template <std::size_t L>
class ParticleFilter {
    static_assert(L > 0 && L <= 500, "Particle count must be between 1 and 500");

private:
    std::array<Vec2, L> particles{};
    std::array<Vec2, L> oldParticles{};
    std::array<double, L> weights{};
    std::vector<DistanceSensor*> sensors;
    FieldModel& field;
    std::function<float()> headingFunction;
    std::ranlux24_base randomEngine;

    Pose prediction{};
    float distanceSinceUpdate = 0.0f;
    std::uint32_t lastUpdateTime = 0;
    bool healthy = true;
    bool correctionFresh = false;

    /** Recomputes the arithmetic particle mean and attaches the current IMU heading. */
    void updateMean() {
        float xSum = 0.0f;
        float ySum = 0.0f;
        for (const Vec2& particle : particles) {
            xSum += particle.x;
            ySum += particle.y;
        }
        prediction.x = xSum / static_cast<float>(L);
        prediction.y = ySum / static_cast<float>(L);
        prediction.theta = headingFunction();
    }

    /** Weights and systematically resamples particles, returning whether a correction is fresh. */
    bool resample() {
        correctionFresh = false;
        std::vector<bool> usableSensors(sensors.size(), false);
        std::size_t usableCount = 0;
        for (std::size_t i = 0; i < sensors.size(); ++i) {
            if (sensors[i]->isValid()) {
                usableSensors[i] = true;
                ++usableCount;
            }
        }
        if (usableCount == 0) {
            healthy = true;
            lastUpdateTime = pros::millis();
            distanceSinceUpdate = 0.0f;
            return false;
        }

        double totalWeight = 0.0;
        std::uniform_real_distribution<float> fieldDistribution(-field.halfFieldExtent(), field.halfFieldExtent());

        for (std::size_t i = 0; i < L; ++i) {
            if (std::abs(particles[i].x) > field.halfFieldExtent() ||
                std::abs(particles[i].y) > field.halfFieldExtent()) {
                particles[i] = {fieldDistribution(randomEngine), fieldDistribution(randomEngine)};
            }

            double weight = 1.0;
            const Pose particlePose{particles[i].x, particles[i].y, headingFunction()};
            for (std::size_t sensorIndex = 0; sensorIndex < sensors.size(); ++sensorIndex) {
                if (usableSensors[sensorIndex]) {
                    weight *= sensors[sensorIndex]->likelihood(particlePose, field);
                }
            }
            weights[i] = weight;
            totalWeight += weight;
        }

        if (!std::isfinite(totalWeight) || totalWeight <= 0.00000000000000000001) {
            healthy = false;
            return false;
        }

        oldParticles = particles;
        const double averageWeight = totalWeight / static_cast<double>(L);
        std::uniform_real_distribution<double> startDistribution(0.0, averageWeight);
        const double start = startDistribution(randomEngine);
        double cumulativeWeight = weights[0];
        std::size_t sourceIndex = 0;

        for (std::size_t i = 0; i < L; ++i) {
            const double target = start + static_cast<double>(i) * averageWeight;
            while (sourceIndex + 1 < L && cumulativeWeight < target) {
                ++sourceIndex;
                cumulativeWeight += weights[sourceIndex];
            }
            particles[i] = oldParticles[sourceIndex];
        }

        updateMean();
        healthy = std::isfinite(prediction.x) && std::isfinite(prediction.y) && std::isfinite(prediction.theta);
        correctionFresh = healthy;
        lastUpdateTime = pros::millis();
        distanceSinceUpdate = 0.0f;
        return healthy;
    }

public:
    /**
     * Supplies the field map and heading source.
     * @param field Game-specific field geometry used by distance sensors.
     * @param headingFunction Callback returning standard CCW heading in degrees.
     */
    explicit ParticleFilter(FieldModel& field, std::function<float()> headingFunction)
        : field(field), headingFunction(std::move(headingFunction)) {}

    /**
     * Registers a distance sensor whose lifetime exceeds the filter's lifetime.
     * @param sensor Non-owning pointer to the sensor adapter; nullptr is ignored.
     */
    void addSensor(DistanceSensor* sensor) {
        if (sensor != nullptr) sensors.push_back(sensor);
    }

    /**
     * Seeds a Gaussian distribution around a known start/reset pose.
     * @param mean Mean x/y in inches and standard CCW heading in degrees.
     * @param stdDevXIn Initial x standard deviation in inches.
     * @param stdDevYIn Initial y standard deviation in inches.
     */
    void initNormal(const Pose& mean, float stdDevXIn, float stdDevYIn) {
        std::normal_distribution<float> distribution(0.0f, 1.0f);
        for (Vec2& particle : particles) {
            particle.x = mean.x + stdDevXIn * distribution(randomEngine);
            particle.y = mean.y + stdDevYIn * distribution(randomEngine);
        }
        prediction = mean;
        healthy = true;
        correctionFresh = false;
        distanceSinceUpdate = MAX_DISTANCE_SINCE_UPDATE_IN;
        lastUpdateTime = pros::millis();
    }

    /**
     * Seeds across the whole field when the starting position is unknown, as Echo does.
     * @param headingSource Pose whose heading supplies the initial heading callback value.
     */
    void initUniform(const Pose& headingSource) {
        std::uniform_real_distribution<float> distribution(-field.halfFieldExtent(), field.halfFieldExtent());
        for (Vec2& particle : particles) {
            particle = {distribution(randomEngine), distribution(randomEngine)};
        }
        prediction = headingSource;
        updateMean();
        healthy = true;
        correctionFresh = false;
        distanceSinceUpdate = MAX_DISTANCE_SINCE_UPDATE_IN;
        lastUpdateTime = pros::millis();
    }

    /**
     * Rotates each low-noise forward delta by a high-trust current IMU heading.
     * @param wheelDeltaIn Tracking-wheel travel since the previous update, in inches.
     */
    void update(float wheelDeltaIn) {
        const float currentHeading = headingFunction();
        if (!std::isfinite(currentHeading)) {
            healthy = false;
            correctionFresh = false;
            return;
        }

        std::uniform_real_distribution<float> driveDistribution(
            -DRIVE_NOISE_FRACTION * std::abs(wheelDeltaIn),
            DRIVE_NOISE_FRACTION * std::abs(wheelDeltaIn));
        std::uniform_real_distribution<float> angleDistribution(-ANGLE_NOISE_DEG, ANGLE_NOISE_DEG);

        for (Vec2& particle : particles) {
            const float noisyWheelDelta = wheelDeltaIn + driveDistribution(randomEngine);
            const float noisyHeading = currentHeading + angleDistribution(randomEngine);
            const Vec2 globalDelta = rotate({noisyWheelDelta, 0.0f}, noisyHeading);
            particle.x += globalDelta.x;
            particle.y += globalDelta.y;
        }

        distanceSinceUpdate += std::abs(wheelDeltaIn);
        updateMean();

        const bool distanceTrigger = distanceSinceUpdate >= MAX_DISTANCE_SINCE_UPDATE_IN;
        const bool timeTrigger = pros::millis() - lastUpdateTime >= MAX_UPDATE_INTERVAL_MS;
        if (distanceTrigger || timeTrigger) {
            for (DistanceSensor* sensor : sensors) sensor->update();
            resample();
        }
    }

    /** @return Latest MCL pose in inches and standard CCW degrees, without changing state. */
    Pose getPrediction() const { return prediction; }
    /** @return True when the last sensor-weighted update produced a usable estimate. */
    bool isHealthy() const { return healthy; }

    /** @return True once for each fresh, healthy correction; false when none is available. */
    bool consumeCorrection() {
        const bool fresh = correctionFresh && healthy;
        correctionFresh = false;
        return fresh;
    }
};

} // namespace mcl
