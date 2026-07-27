#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include <optional>
#include <cstdint>
enum class GameObjectType : uint8_t
{
    BLUE_BLUE_PIN,
    RED_RED_PIN,
    BLUE_YELLOW_PIN,
    RED_YELLOW_PIN,
    YELLOW_YELLOW_PIN,
    HALF_BLUE_PIN,    // <-- new
    HALF_RED_PIN,     // <-- new
    HALF_YELLOW_PIN,  // <-- new
    ANY_PIN,
};
enum class DriveToResult : uint8_t
{
    SUCCESS,
    OBJECT_NOT_FOUND,
    OBJECT_LOST,
    TIMEOUT,
    ABORTED,
};

struct TagReading
{
    int id;
    double distance;       // In whatever unit kTagSize is set to
    double bearing_deg;    // Positive = right of center
    double elevation_deg;  // Positive = above center
    double global_heading; // IMU heading + bearing
};

struct VisionDriveConfig
{
    // Steering PID
    double lateral_kP = 0.5;
    double lateral_kI = 0.00;
    double lateral_kD = 1;

    // Forward speeds (0-127)
    double approach_speed = 90.0;
    double slow_approach_speed = 50.0;
    double creep_speed = 15.0;

    // Object height thresholds (pixels) — bigger = closer
    double far_threshold = 50.0;
    double mid_threshold = 110.0;
    double close_threshold = 150.0;
    double arrival_threshold = 200.0;

    // Steering
    int frame_center_x = 160;
    int centering_deadband = 12;

    // Timeouts
    int timeout_ms = 8000;
    int lost_object_timeout_ms = 500;
    int settle_time_ms = 300;

    // How close odometry gets before vision takes over (inches)
    //double switch_to_vision_distance = 15.0;

    // Reject tiny noise detections
    double min_object_area = 80.0;

    // Toggle rejection — W/H >= 6.0 is a wall toggle, not a pin
    double toggle_min_wh_ratio = 4.0;
};

class VisionDriveController
{
public:
    VisionDriveController(
        lemlib::Chassis &chassis,
        pros::AIVision &ai_vision,
        pros::Imu &imu,
        pros::Motor &intake_motor,
        double cam_fwd_in = 5.0,
        double cam_height_in = 8.0,
        double cam_pitch_deg = -30.0,
        VisionDriveConfig config = {});

    /// Register colors, codes, and tag family. Call once in initialize().
    void setupSensor();

    /// Drive to a known game piece at (target_x, target_y) on the field.

    DriveToResult acquireAlliancePin(
    int x,
    int y,
    int visionSwitchDistance,
    bool pivotIntake,
    int timeout,
    bool isYellow = false);  // <-- new param
    /**
     * Get distance/bearing to a visible AprilTag.
     * @param tag_id  If >= 0, only match this tag. If -1, return closest.
     * @return        TagReading if found, std::nullopt otherwise.
     */
    std::optional<TagReading> getTagReading(int tag_id = -1);

    void abort();
    void setConfig(const VisionDriveConfig &config);
    bool isRunning() const;

private:
    lemlib::Chassis &m_chassis;
    pros::AIVision &m_ai_vision;
    pros::Imu &m_imu;
    pros::Motor &m_intake_motor;

    double m_cam_fwd_in;
    double m_cam_height_in;
    double m_cam_pitch_deg;

    VisionDriveConfig m_config;

    bool m_running = false;
    bool m_abort_flag = false;

    DriveToResult visionApproachLoop(GameObjectType object_type, bool run_intake);
    std::optional<pros::AIVision::Object> findBestTarget(GameObjectType type);
    bool matchesTarget(const pros::AIVision::Object &obj, GameObjectType type) const;
    bool isValidDetection(const pros::AIVision::Object &obj) const;
    bool isProbablyToggle(const pros::AIVision::Object &obj) const;
    double calculateApproachSpeed(const pros::AIVision::Object &obj) const;
    void arcadeDrive(double throttle, double turn);
    void stopDrive();
};