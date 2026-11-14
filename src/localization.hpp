#pragma once

#include "globals.hpp"
#include "localization/mcl.hpp"
#include "localization/override_field.hpp"
#include "pros/rtos.hpp"

#include <array>
#include <cmath>

/**
 * Owns the four distance sensors, MCL task, and guarded LemLib pose bridge.
 * Corrections are optional and disabled by default; LemLib odometry always remains
 * the fallback and its motion queue is never interrupted by this class.
 */
class Localization {
private:
    // Provisional beam height: above the 3.25-inch short goals, below the 8.77-inch center goal.

    mcl::OverrideField field;
    std::array<mcl::DistanceSensor, 4> wallSensors{
        mcl::DistanceSensor({&frontDistSensor, FRONT_DIST_OFFSET, 3, 10, 0.0f, 1.0f, 0.25f, true}),
        mcl::DistanceSensor({&backDistSensor, -BACK_DIST_OFFSET, 2.5, 8, 180.0f, 1.0f, 0.25f, true}),
        mcl::DistanceSensor({&leftDistSensor, 2, LEFT_DIST_OFFSET, 8, 90.0f, 1.0f, 0.25f, true}),
        mcl::DistanceSensor({&rightDistSensor, 1.5, -RIGHT_DIST_OFFSET, 8, -90.0f, 1.0f, 0.25f, true}),
    };
    mcl::ParticleFilter<250> particleFilter;
    pros::Task* task = nullptr;
    pros::Mutex mutex;
    float previousWheelIn = 0.0f;
    float previousHeadingDeg = 0.0f;
    mcl::Pose latestPose{};
    lemlib::Pose trackingPose{0.0f, 0.0f, 0.0f};

    /** Converts LemLib's clockwise compass heading to standard CCW degrees. */
    static float toMclHeading(float lemlibHeadingDeg) {
        return 90.0f - lemlibHeadingDeg;
    }

    /** Converts standard CCW degrees back to LemLib's clockwise compass heading. */
    static float toLemlibHeading(float mclHeadingDeg) {
        return 90.0f - mclHeadingDeg;
    }

    /** Integrates a separate wheel/IMU-only pose for comparison against MCL telemetry. */
    void updateTrackingPose(float wheelDeltaIn, float headingDeltaMclDeg, float currentHeadingMclDeg) {
        const float currentHeadingDeg = toLemlibHeading(currentHeadingMclDeg);
        const float headingDeltaDeg = -headingDeltaMclDeg;
        const float previousHeadingDeg = currentHeadingDeg - headingDeltaDeg;
        const float deltaTheta = headingDeltaDeg * mcl::DEG_TO_RAD;

        float forwardChordIn = wheelDeltaIn;
        if (std::abs(deltaTheta) > 0.000001f) {
            forwardChordIn = 2.0f * std::sin(deltaTheta / 2.0f) *
                             (wheelDeltaIn / deltaTheta + vertical_tracking_wheel.getOffset());
        }

        const float averageHeading = (previousHeadingDeg + headingDeltaDeg / 2.0f) * mcl::DEG_TO_RAD;
        trackingPose.x += forwardChordIn * std::sin(averageHeading);
        trackingPose.y += forwardChordIn * std::cos(averageHeading);
        trackingPose.theta = currentHeadingDeg;
    }

    /** Propagates MCL and applies each fresh, valid correction when enabled. */
    void update() {
        mutex.lock();
        const float wheelIn = vertical_tracking_wheel.getDistanceTraveled();
        const float headingDeg = toMclHeading(chassis.getPose().theta);
        const float wheelDeltaIn = wheelIn - previousWheelIn;
        const float headingDeltaDeg = mcl::angleDifference(headingDeg, previousHeadingDeg);
        previousWheelIn = wheelIn;
        previousHeadingDeg = headingDeg;

        updateTrackingPose(wheelDeltaIn, headingDeltaDeg, headingDeg);
        particleFilter.update(wheelDeltaIn);
        latestPose = particleFilter.getPrediction();

        if (particleFilter.consumeCorrection() && enableMclCorrections && particleFilter.isHealthy() &&
            std::isfinite(latestPose.x) && std::isfinite(latestPose.y) && std::isfinite(latestPose.theta)) {
            chassis.setPose(latestPose.x, latestPose.y, toLemlibHeading(latestPose.theta));
        }
        mutex.unlock();
    }

public:
    Localization() : particleFilter(field, [this] { return toMclHeading(chassis.getPose().theta); }) {
        for (mcl::DistanceSensor& sensor : wallSensors) particleFilter.addSensor(&sensor);
    }

    /** Preserves the configured starting pose and starts Echo's 10 ms MCL update period. */
    void start() {
        const lemlib::Pose pose = chassis.getPose();
        resetPose(pose.x, pose.y, pose.theta);
        if (task == nullptr) {
            task = new pros::Task([this] {
                while (true) {
                    update();
                    pros::delay(20);
                }
            }, "MCL Localization");
        }
    }

    /** Deliberately scatters particles across the field when the current position is unknown. */
    void globalRelocalize() {
        mutex.lock();
        const lemlib::Pose pose = chassis.getPose();
        latestPose = {pose.x, pose.y, toMclHeading(pose.theta)};
        previousWheelIn = vertical_tracking_wheel.getDistanceTraveled();
        previousHeadingDeg = latestPose.theta;
        particleFilter.initUniform(latestPose);
        latestPose = particleFilter.getPrediction();
        mutex.unlock();
    }

    /**
     * Atomically resets LemLib and the particles so stale MCL data cannot undo a reset.
     * @param x New field x coordinate in inches.
     * @param y New field y coordinate in inches.
     * @param theta New LemLib compass heading in degrees.
     */
    void resetPose(float x, float y, float theta) {
        mutex.lock();
        chassis.setPose(x, y, theta);
        previousWheelIn = vertical_tracking_wheel.getDistanceTraveled();
        previousHeadingDeg = toMclHeading(theta);
        latestPose = {x, y, previousHeadingDeg};
        trackingPose = {x, y, theta};
        particleFilter.initNormal(latestPose, 1.0f, 1.0f);
        mutex.unlock();
    }

    /** @return MCL estimate converted to LemLib's inches/compass-degree convention. */
    lemlib::Pose getCorrectedPose() {
        mutex.lock();
        const lemlib::Pose pose{latestPose.x, latestPose.y, toLemlibHeading(latestPose.theta)};
        mutex.unlock();
        return pose;
    }

    /** @return Independent tracking-wheel/IMU pose without MCL corrections. */
    lemlib::Pose getTrackingPose() {
        mutex.lock();
        const lemlib::Pose pose = trackingPose;
        mutex.unlock();
        return pose;
    }

};

inline Localization localization;
