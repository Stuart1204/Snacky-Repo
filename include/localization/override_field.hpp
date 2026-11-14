#pragma once

#include "localization/mcl.hpp"

#include <array>
#include <cmath>
#include <limits>

/**
 * Override v2.1 field geometry used by the MCL distance-sensor model.
 * All coordinates and dimensions are inches; all beam angles are degrees.
 */
namespace mcl {

/** Ray-casts against walls, the tall center goal, and masked loader regions. */
class OverrideField final : public FieldModel {
public:
    static constexpr float HALF_FIELD_EXTENT_IN = 140.4f / 2.0f;
    static constexpr float TALL_GOAL_HEIGHT_IN = 8.77f;
    static constexpr float LOADER_MAX_HEIGHT_IN = 22.97f;

private:
    /** Fixed goal height, bounding radius, and exact ray-casting vertices. */
    struct Goal {
        Vec2 center;
        float heightIn;
        float boundingRadiusIn;
        std::array<Vec2, 8> vertices;
    };

    std::array<Goal, 1> goals;

    /** Builds an octagonal goal footprint around a manual-derived center coordinate. */
    static Goal makeGoal(float x, float y, float heightIn, float circumradiusIn) {
        Goal goal{};
        goal.center = {x, y};
        goal.heightIn = heightIn;
        goal.boundingRadiusIn = circumradiusIn;
        for (std::size_t i = 0; i < goal.vertices.size(); ++i) {
            const float angle = (22.5f + 45.0f * static_cast<float>(i)) * DEG_TO_RAD;
            goal.vertices[i] = {x + circumradiusIn * std::cos(angle),
                                y + circumradiusIn * std::sin(angle)};
        }
        return goal;
    }

    /** Computes the 2D cross product used by ray/segment intersection tests. */
    static float cross(Vec2 first, Vec2 second) {
        return first.x * second.y - first.y * second.x;
    }

    /** Returns a forward ray/segment distance, or infinity when there is no intersection. */
    static float raySegmentDistance(Vec2 origin, Vec2 direction, Vec2 start, Vec2 end) {
        const Vec2 segment{end.x - start.x, end.y - start.y};
        const Vec2 relative{start.x - origin.x, start.y - origin.y};
        const float denominator = cross(direction, segment);
        if (std::abs(denominator) < 0.000001f) return std::numeric_limits<float>::infinity();

        const float rayDistance = cross(relative, segment) / denominator;
        const float segmentPosition = cross(relative, direction) / cross(segment, direction);
        if (rayDistance >= 0.0f && segmentPosition >= 0.0f && segmentPosition <= 1.0f) {
            return rayDistance;
        }
        return std::numeric_limits<float>::infinity();
    }

    /** Finds the nearest ray intersection with one goal's octagonal boundary. */
    float goalDistance(const Goal& goal, Vec2 origin, Vec2 direction) const {
        const Vec2 relative{goal.center.x - origin.x, goal.center.y - origin.y};
        const float projection = relative.x * direction.x + relative.y * direction.y;
        const float closestSquared = relative.x * relative.x + relative.y * relative.y - projection * projection;
        if (projection < -goal.boundingRadiusIn ||
            closestSquared > goal.boundingRadiusIn * goal.boundingRadiusIn) {
            return std::numeric_limits<float>::infinity();
        }

        float nearest = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < goal.vertices.size(); ++i) {
            const std::size_t next = (i + 1) % goal.vertices.size();
            nearest = std::min(nearest, raySegmentDistance(origin, direction, goal.vertices[i], goal.vertices[next]));
        }
        return nearest;
    }

    /** Masks corner wall hits where loader geometry can differ from a flat wall. */
    static bool isLoaderWallHit(Vec2 hit, float sensorHeightIn) {
        return sensorHeightIn <= LOADER_MAX_HEIGHT_IN &&
               std::abs(hit.x) >= HALF_FIELD_EXTENT_IN - 12.0f &&
               std::abs(hit.y) >= HALF_FIELD_EXTENT_IN - 12.0f;
    }

    /** Finds the nearest positive intersection with the four inner perimeter walls. */
    ExpectedDistance nearestWall(Vec2 origin, Vec2 direction) const {
        float nearest = std::numeric_limits<float>::infinity();
        Vec2 hit{};

        const auto consider = [&](float distance, bool& valid) {
            if (distance <= 0.0001f || distance >= nearest) return;
            const Vec2 candidate{origin.x + direction.x * distance, origin.y + direction.y * distance};
            if (std::abs(candidate.x) <= HALF_FIELD_EXTENT_IN + 0.01f &&
                std::abs(candidate.y) <= HALF_FIELD_EXTENT_IN + 0.01f) {
                nearest = distance;
                hit = candidate;
                valid = true;
            }
        };

        bool valid = false;
        if (std::abs(direction.x) > 0.000001f) {
            consider((HALF_FIELD_EXTENT_IN - origin.x) / direction.x, valid);
            consider((-HALF_FIELD_EXTENT_IN - origin.x) / direction.x, valid);
        }
        if (std::abs(direction.y) > 0.000001f) {
            consider((HALF_FIELD_EXTENT_IN - origin.y) / direction.y, valid);
            consider((-HALF_FIELD_EXTENT_IN - origin.y) / direction.y, valid);
        }
        return {nearest, valid};
    }

public:
    /** Models only the tall center goal because the sensors are mounted above the other goals. */
    OverrideField() {
        const float tallRadius = 5.52f / (2.0f * std::cos(22.5f * DEG_TO_RAD));
        goals = {makeGoal(0.0f, 0.0f, TALL_GOAL_HEIGHT_IN, tallRadius)};
    }

    /**
     * Returns the nearest usable wall/center-goal ray.
     * @param particle Candidate field pose in inches/degrees.
     * @param sensor Sensor position, height, yaw, and calibration fields.
     * @return Predicted distance in inches and whether this ray may affect weights.
     */
    ExpectedDistance expectedDistance(const Pose& particle, const DistanceConfig& sensor) const override {
        const Vec2 sensorOffset = rotate({sensor.forwardIn, sensor.leftIn}, particle.theta);
        const Vec2 origin{particle.x + sensorOffset.x, particle.y + sensorOffset.y};
        const float beamAngle = (particle.theta + sensor.yawDeg) * DEG_TO_RAD;
        const Vec2 direction{std::cos(beamAngle), std::sin(beamAngle)};

        ExpectedDistance nearest = nearestWall(origin, direction);
        Vec2 wallHit{};
        if (nearest.usable) {
            wallHit = {origin.x + direction.x * nearest.distanceIn,
                       origin.y + direction.y * nearest.distanceIn};
        }

        for (const Goal& goal : goals) {
            if (sensor.heightIn > goal.heightIn) continue;
            const float distance = goalDistance(goal, origin, direction);
            if (distance < nearest.distanceIn) nearest = {distance, true};
        }

        if (nearest.usable && nearest.distanceIn > 0.0f) {
            const bool usedWall = wallHit.x != 0.0f || wallHit.y != 0.0f;
            const Vec2 nearestHit{origin.x + direction.x * nearest.distanceIn,
                                  origin.y + direction.y * nearest.distanceIn};
            if (usedWall && nearestHit.x == wallHit.x && nearestHit.y == wallHit.y &&
                isLoaderWallHit(wallHit, sensor.heightIn)) {
                nearest.usable = false;
            }
        }
        return nearest;
    }

    /** @return Override's nominal center-to-inner-wall distance in inches. */
    float halfFieldExtent() const override { return HALF_FIELD_EXTENT_IN; }
};

} // namespace mcl
