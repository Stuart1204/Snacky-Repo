#pragma once

#include "api.h"
#include "lemlib/api.hpp"
#include <optional>
#include <cstdint>

enum class GameObjectType : uint8_t {
    BLUE_BLUE_PIN,
    RED_RED_PIN,
    BLUE_YELLOW_PIN,
    RED_YELLOW_PIN,
    YELLOW_YELLOW_PIN,
    ANY_PIN,
};

enum class DriveToResult : uint8_t {
    SUCCESS,
    OBJECT_NOT_FOUND,
    OBJECT_LOST,
    TIMEOUT,
    ABORTED,
};

struct VisionDriveConfig {
    // Steering PID
    double lateral_kP = 5.6;
    double lateral_kI = 0.00;
    double lateral_kD = 29;

    // Forward speeds (0-127)
    double approach_speed      = 70.0;
    double slow_approach_speed = 50.0;
    double creep_speed         = 30.0;

    // Object height thresholds (pixels) — bigger = closer
    double far_threshold     =  50.0;
    double mid_threshold     = 100.0;
    double close_threshold   = 160.0;
    double arrival_threshold = 190.0;

    // Steering
    int frame_center_x     = 160;
    int centering_deadband =  12;

    // Timeouts
    int timeout_ms             = 8000;
    int lost_object_timeout_ms = 500;
    int settle_time_ms         = 300;

    // How close odometry gets before vision takes over (inches)
    double switch_to_vision_distance = 12.0;

    // Reject tiny noise detections
    double min_object_area = 80.0;
};

class VisionDriveController {
public:
    /**
     * @param chassis      LemLib chassis
     * @param ai_vision    PROS AIVision sensor
     * @param intake_motor Intake motor
     * @param cam_fwd_in   Camera forward offset from tracking center (inches)
     * @param cam_height_in Camera height above ground (inches)
     * @param cam_pitch_deg Camera pitch (negative = looking down)
     * @param config       Tuning parameters
     */
    VisionDriveController(
        lemlib::Chassis&   chassis,
        pros::AIVision&    ai_vision,
        pros::Motor&       intake_motor,
        double             cam_fwd_in    =  5.0,
        double             cam_height_in =  8.0,
        double             cam_pitch_deg = -30.0,
        VisionDriveConfig  config        = {}
    );

    /// Register all color signatures and codes. Call once in initialize().
    void setupSensorColors();

    /// Drive to a known game piece at (target_x, target_y) on the field.
    DriveToResult driveToObject(
        double         target_x,
        double         target_y,
        GameObjectType object_type,
        bool           run_intake = true
    );

    void abort();
    void setConfig(const VisionDriveConfig& config);
    bool isRunning() const;

private:
    lemlib::Chassis&   m_chassis;
    pros::AIVision&    m_ai_vision;
    pros::Motor&       m_intake_motor;

    double m_cam_fwd_in;
    double m_cam_height_in;
    double m_cam_pitch_deg;

    VisionDriveConfig m_config;

    bool m_running    = false;
    bool m_abort_flag = false;

    /// Vision tracking loop — steer to keep object centered, drive forward
    DriveToResult visionApproachLoop(GameObjectType object_type, bool run_intake);

    /// Find largest matching object in current frame
    std::optional<pros::AIVision::Object> findBestTarget(GameObjectType type);

    /// Does this detection match the requested game object?
    bool matchesTarget(const pros::AIVision::Object& obj, GameObjectType type) const;

    /// Reject noise / garbage detections
    bool isValidDetection(const pros::AIVision::Object& obj) const;

    /// Forward speed interpolated from object apparent height
    double calculateApproachSpeed(const pros::AIVision::Object& obj) const;

    /// Direct arcade-style drive output for vision control
    void arcadeDrive(double throttle, double turn);

    /// Stop chassis motion immediately
    void stopDrive();
};