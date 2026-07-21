#include "vision_drive.hpp"
#include <algorithm>
#include <cmath>

// ============================================================================
// CONSTANTS
// ============================================================================

static constexpr double kImageW = 320.0;
static constexpr double kImageH = 240.0;
static constexpr double kHFovDeg = 75.0;
static constexpr double kPi = 3.14159265358979323846;
static constexpr double kDegToRad = kPi / 180.0;
static const double kFx = (kImageW / 2.0) / std::tan((kHFovDeg * kDegToRad) / 2.0);
static const double kFy = kFx;
static const double kCx = kImageW / 2.0;
static const double kCy = kImageH / 2.0;

// AprilTag physical side length — set to your real tag size.
// Returned distance from getTagReading() will be in the same unit.
static constexpr double kTagSize = 6.0; // inches

// ============================================================================
// OBJECT ACCESSORS
// ============================================================================

static int getObjectX(const pros::AIVision::Object &obj)
{
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object))
    {
        return static_cast<int>(obj.object.element.xoffset + obj.object.element.width / 2);
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag))
    {
        return (obj.object.tag.x0 + obj.object.tag.x1 + obj.object.tag.x2 + obj.object.tag.x3) / 4;
    }
    return static_cast<int>(obj.object.color.xoffset + obj.object.color.width / 2);
}

static int getObjectY(const pros::AIVision::Object &obj)
{
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object))
    {
        return static_cast<int>(obj.object.element.yoffset + obj.object.element.height / 2);
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag))
    {
        return (obj.object.tag.y0 + obj.object.tag.y1 + obj.object.tag.y2 + obj.object.tag.y3) / 4;
    }
    return static_cast<int>(obj.object.color.yoffset + obj.object.color.height / 2);
}

static int getObjectWidth(const pros::AIVision::Object &obj)
{
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object))
    {
        return static_cast<int>(obj.object.element.width);
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag))
    {
        return std::abs(obj.object.tag.x1 - obj.object.tag.x0);
    }
    return static_cast<int>(obj.object.color.width);
}

static int getObjectHeight(const pros::AIVision::Object &obj)
{
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object))
    {
        return static_cast<int>(obj.object.element.height);
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag))
    {
        return std::abs(obj.object.tag.y1 - obj.object.tag.y0);
    }
    return static_cast<int>(obj.object.color.height);
}

// ============================================================================
// SENSOR IDs
// ============================================================================

namespace ColorIDs
{
    constexpr int BLUE = 1;
    constexpr int YELLOW = 2;
    constexpr int RED = 3;
}

namespace CodeIDs
{
    constexpr int YELLOW_YELLOW = 1;
    constexpr int RED_YELLOW = 2;
    constexpr int BLUE_YELLOW = 3;
    constexpr int BLUE_BLUE = 4;
    constexpr int RED_RED = 5;
}

// ============================================================================
// CONSTRUCTOR
// ============================================================================

VisionDriveController::VisionDriveController(
    lemlib::Chassis &chassis,
    pros::AIVision &ai_vision,
    pros::Imu &inertial_sensor,
    pros::Motor &intake_motor,
    double cam_fwd_in,
    double cam_height_in,
    double cam_pitch_deg,
    VisionDriveConfig config)
    : m_chassis(chassis), m_ai_vision(ai_vision), m_imu(inertial_sensor), m_intake_motor(intake_motor), m_cam_fwd_in(cam_fwd_in), m_cam_height_in(cam_height_in), m_cam_pitch_deg(cam_pitch_deg), m_config(config)
{
}

// ============================================================================
// SENSOR SETUP
// ============================================================================

void VisionDriveController::setupSensor()
{
    m_ai_vision.reset();
    pros::delay(100);

    // Enable both color/code AND tag detection
    m_ai_vision.enable_detection_types(
        pros::AivisionModeType::colors,
        pros::AivisionModeType::tags);
    m_ai_vision.set_tag_family(pros::AivisionTagFamily::tag_21H7);

    // --- Colors (from VEXcode robot config) ---

    pros::AIVision::Color blue = {
        .id = ColorIDs::BLUE,
        .red = 77,
        .green = 150,
        .blue = 180,
        .hue_range = 11.0,
        .saturation_range = 0.25};
    m_ai_vision.set_color(blue);

    pros::AIVision::Color yellow = {
        .id = ColorIDs::YELLOW,
        .red = 139,
        .green = 147,
        .blue = 99,
        .hue_range = 26.0,
        .saturation_range = 0.33};
    m_ai_vision.set_color(yellow);

    pros::AIVision::Color red = {
        .id = ColorIDs::RED,
        .red = 174,
        .green = 82,
        .blue = 109,
        .hue_range = 11.0,
        .saturation_range = 0.68};
    m_ai_vision.set_color(red);

    // --- Two-color codes ---

    pros::AIVision::Code yel_yel = {
        .id = CodeIDs::YELLOW_YELLOW, .length = 2, .c1 = ColorIDs::YELLOW, .c2 = ColorIDs::YELLOW};
    m_ai_vision.set_code(yel_yel);

    pros::AIVision::Code red_yel = {
        .id = CodeIDs::RED_YELLOW, .length = 2, .c1 = ColorIDs::RED, .c2 = ColorIDs::YELLOW};
    m_ai_vision.set_code(red_yel);

    pros::AIVision::Code blue_yel = {
        .id = CodeIDs::BLUE_YELLOW, .length = 2, .c1 = ColorIDs::BLUE, .c2 = ColorIDs::YELLOW};
    m_ai_vision.set_code(blue_yel);

    pros::AIVision::Code blue_blue = {
        .id = CodeIDs::BLUE_BLUE, .length = 2, .c1 = ColorIDs::BLUE, .c2 = ColorIDs::BLUE};
    m_ai_vision.set_code(blue_blue);

    pros::AIVision::Code red_red = {
        .id = CodeIDs::RED_RED, .length = 2, .c1 = ColorIDs::RED, .c2 = ColorIDs::RED};
    m_ai_vision.set_code(red_red);

    pros::delay(50);
}

// ============================================================================
// PUBLIC
// ============================================================================

void VisionDriveController::abort() { m_abort_flag = true; }
void VisionDriveController::setConfig(const VisionDriveConfig &config) { m_config = config; }
bool VisionDriveController::isRunning() const { return m_running; }

// ============================================================================
// APRIL TAG READING
// ============================================================================

std::optional<TagReading> VisionDriveController::getTagReading(int tag_id)
{
    auto objects = m_ai_vision.get_all_objects();
    if (objects.empty())
        return std::nullopt;

    std::optional<TagReading> best;
    double best_dist = 1e9;

    for (const auto &obj : objects)
    {
        if (!pros::AIVision::is_type(obj, pros::AivisionDetectType::tag))
            continue;

        if (tag_id >= 0 && obj.id != tag_id)
            continue;

        double x0 = obj.object.tag.x0, y0 = obj.object.tag.y0;
        double x1 = obj.object.tag.x1, y1 = obj.object.tag.y1;
        double x2 = obj.object.tag.x2, y2 = obj.object.tag.y2;
        double x3 = obj.object.tag.x3, y3 = obj.object.tag.y3;

        auto edgeDist = [](double ax, double ay, double bx, double by)
        {
            double dx = bx - ax, dy = by - ay;
            return std::sqrt(dx * dx + dy * dy);
        };

        double avg_edge = (edgeDist(x0, y0, x1, y1) +
                           edgeDist(x1, y1, x2, y2) +
                           edgeDist(x2, y2, x3, y3) +
                           edgeDist(x3, y3, x0, y0)) /
                          4.0;

        if (avg_edge <= 0.0)
            continue;

        double distance = (kTagSize * kFx) / avg_edge;

        double cx = (x0 + x1 + x2 + x3) / 4.0;
        double cy = (y0 + y1 + y2 + y3) / 4.0;

        double bearing = std::atan2(cx - kCx, kFx) * (180.0 / kPi);
        double elevation = std::atan2(kCy - cy, kFy) * (180.0 / kPi);

        double global_heading = m_imu.get_heading() + bearing;

        if (distance < best_dist)
        {
            best_dist = distance;
            best = TagReading{
                .id = obj.id,
                .distance = distance,
                .bearing_deg = bearing,
                .elevation_deg = elevation,
                .global_heading = global_heading};
        }
    }

    return best;
}

// ============================================================================
// DRIVE TO OBJECT AT KNOWN COORDINATES
// ============================================================================

DriveToResult VisionDriveController::driveToObject(
    double target_x,
    double target_y,
    GameObjectType object_type,
    bool run_intake)
{
    m_running = true;
    m_abort_flag = false;

    // Phase 1 — Odometry approach
    // Camera is m_cam_fwd_in ahead of tracking center, so stop short enough
    // that the CAMERA is switch_to_vision_distance from target
    lemlib::Pose cur = m_chassis.getPose();
    double dx = target_x - cur.x;
    double dy = target_y - cur.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double stop_short = m_config.switch_to_vision_distance + m_cam_fwd_in;

    if (dist > stop_short)
    {
        double ratio = (dist - stop_short) / dist;
        double wp_x = cur.x + dx * ratio;
        double wp_y = cur.y + dy * ratio;

        int timeout = static_cast<int>(dist / 20.0 * 1000.0) + 2000;

        m_chassis.moveToPoint(wp_x, wp_y, timeout, {.forwards = true, .maxSpeed = static_cast<float>(m_config.approach_speed), .minSpeed = 0.0f, .earlyExitRange = static_cast<float>(stop_short * 0.3)});

        if (m_abort_flag)
        {
            stopDrive();
            m_running = false;
            return DriveToResult::ABORTED;
        }
    }

    // Phase 2 — Face target
    m_chassis.turnToPoint(target_x, target_y, 1500, {.forwards = true, .minSpeed = 0});

    if (m_abort_flag)
    {
        stopDrive();
        m_running = false;
        return DriveToResult::ABORTED;
    }

    // Phase 3 — Vision approach
    DriveToResult result = visionApproachLoop(object_type, run_intake);
    m_running = false;
    return result;
}

// ============================================================================
// VISION APPROACH LOOP
// ============================================================================

DriveToResult VisionDriveController::visionApproachLoop(
    GameObjectType object_type,
    bool run_intake)
{
    uint32_t start_ms = pros::millis();
    uint32_t last_seen_ms = pros::millis();
    uint32_t arrival_start = 0;

    bool ever_found = false;
    bool at_arrival = false;

    double prev_error = 0.0;
    double integral = 0.0;

    while (true)
    {
        if (m_abort_flag)
        {
            stopDrive();
            if (run_intake)
                m_intake_motor.move(0);
            return DriveToResult::ABORTED;
        }

        if (pros::millis() - start_ms > static_cast<uint32_t>(m_config.timeout_ms))
        {
            stopDrive();
            if (run_intake)
                m_intake_motor.move(0);
            return ever_found ? DriveToResult::OBJECT_LOST
                              : DriveToResult::OBJECT_NOT_FOUND;
        }

        auto target = findBestTarget(object_type);

        if (target.has_value())
        {
            const auto &obj = target.value();
            ever_found = true;
            last_seen_ms = pros::millis();

            // ---- Horizontal centering PID ----
            double error = static_cast<double>(getObjectX(obj)) - m_config.frame_center_x;

            integral += error;
            integral = std::clamp(integral, -500.0, 500.0);
            double deriv = error - prev_error;
            if (error * (error - deriv) < 0.0)
                integral = 0.0;
            prev_error = error;

            double turn = m_config.lateral_kP * error + m_config.lateral_kI * integral + m_config.lateral_kD * deriv;
            turn = std::clamp(turn, -60.0, 60.0);

            // ---- Forward speed from apparent height ----
            double forward = calculateApproachSpeed(obj);
            double obj_h = static_cast<double>(getObjectHeight(obj));

            // ---- Arrival ----
            if (obj_h >= m_config.arrival_threshold)
            {
                if (!at_arrival)
                {
                    at_arrival = true;
                    arrival_start = pros::millis();
                }

                if (run_intake)
                    m_intake_motor.move(127);

                if (pros::millis() - arrival_start >= static_cast<uint32_t>(m_config.settle_time_ms))
                {
                    arcadeDrive(m_config.creep_speed, 0.0);
                    pros::delay(200);
                    stopDrive();
                    pros::delay(300);
                    if (run_intake)
                        m_intake_motor.move(0);
                    return DriveToResult::SUCCESS;
                }

                arcadeDrive(m_config.creep_speed, turn * 0.4);
            }
            else
            {
                at_arrival = false;

                if (run_intake && obj_h >= m_config.mid_threshold)
                {
                    m_intake_motor.move(127);
                }

                if (std::abs(error) <= m_config.centering_deadband)
                {
                    arcadeDrive(forward, 0.0);
                }
                else
                {
                    double scale = 1.0 - std::min(std::abs(turn) / 60.0, 0.6);
                    arcadeDrive(forward * scale, turn);
                }
            }
        }
        else
        {
            at_arrival = false;

            if (ever_found)
            {
                if (pros::millis() - last_seen_ms > static_cast<uint32_t>(m_config.lost_object_timeout_ms))
                {
                    stopDrive();
                    if (run_intake)
                        m_intake_motor.move(0);
                    return DriveToResult::OBJECT_LOST;
                }
                arcadeDrive(m_config.creep_speed, 0.0);
            }
            else
            {
                arcadeDrive(0.0, 25.0);
            }
        }

        pros::delay(20);
    }
}

// ============================================================================
// FIND BEST TARGET
// ============================================================================

std::optional<pros::AIVision::Object>
VisionDriveController::findBestTarget(GameObjectType type)
{
    auto objects = m_ai_vision.get_all_objects();
    if (objects.empty())
        return std::nullopt;

    std::optional<pros::AIVision::Object> best;
    int best_area = 0;

    for (const auto &obj : objects)
    {
        if (!isValidDetection(obj))
            continue;
        if (isProbablyToggle(obj))
            continue;
        if (!matchesTarget(obj, type))
            continue;

        int area = getObjectWidth(obj) * getObjectHeight(obj);
        if (area > best_area)
        {
            best_area = area;
            best = obj;
        }
    }
    return best;
}

// ============================================================================
// TARGET MATCHING
// ============================================================================

bool VisionDriveController::matchesTarget(
    const pros::AIVision::Object &obj,
    GameObjectType type) const
{
    bool is_code = pros::AIVision::is_type(obj, pros::AivisionDetectType::code);

    switch (type)
    {
    case GameObjectType::BLUE_BLUE_PIN:
        return is_code && obj.id == CodeIDs::BLUE_BLUE;
    case GameObjectType::RED_RED_PIN:
        return is_code && obj.id == CodeIDs::RED_RED;
    case GameObjectType::BLUE_YELLOW_PIN:
        return is_code && obj.id == CodeIDs::BLUE_YELLOW;
    case GameObjectType::RED_YELLOW_PIN:
        return is_code && obj.id == CodeIDs::RED_YELLOW;
    case GameObjectType::YELLOW_YELLOW_PIN:
        return is_code && obj.id == CodeIDs::YELLOW_YELLOW;
    case GameObjectType::ANY_PIN:
        return is_code;
    default:
        return false;
    }
}

// ============================================================================
// TOGGLE REJECTION
// ============================================================================

bool VisionDriveController::isProbablyToggle(
    const pros::AIVision::Object &obj) const
{
    int h = getObjectHeight(obj);
    if (h <= 0)
        return false;

    double wh_ratio = static_cast<double>(getObjectWidth(obj)) /
                      static_cast<double>(h);

    return wh_ratio >= m_config.toggle_min_wh_ratio;
}

// ============================================================================
// DETECTION FILTER
// ============================================================================

bool VisionDriveController::isValidDetection(
    const pros::AIVision::Object &obj) const
{
    int w = getObjectWidth(obj);
    int h = getObjectHeight(obj);
    double area = static_cast<double>(w * h);
    if (area < m_config.min_object_area)
        return false;

    int x = getObjectX(obj);
    int y = getObjectY(obj);
    if (x < 0 || x >= static_cast<int>(kImageW))
        return false;
    if (y < 0 || y >= static_cast<int>(kImageH))
        return false;
    if (w > static_cast<int>(kImageW) - 10)
        return false;
    if (h > static_cast<int>(kImageH) - 10)
        return false;
    return true;
}

// ============================================================================
// SPEED FROM APPARENT SIZE
// ============================================================================

double VisionDriveController::calculateApproachSpeed(
    const pros::AIVision::Object &obj) const
{
    double h = static_cast<double>(getObjectHeight(obj));

    if (h < m_config.far_threshold)
        return m_config.approach_speed;

    if (h < m_config.mid_threshold)
    {
        double t = (h - m_config.far_threshold) / (m_config.mid_threshold - m_config.far_threshold);
        return m_config.approach_speed + t * (m_config.slow_approach_speed - m_config.approach_speed);
    }

    if (h < m_config.close_threshold)
    {
        double t = (h - m_config.mid_threshold) / (m_config.close_threshold - m_config.mid_threshold);
        return m_config.slow_approach_speed + t * (m_config.creep_speed - m_config.slow_approach_speed);
    }

    return m_config.creep_speed;
}

// ============================================================================
// DRIVETRAIN
// ============================================================================

void VisionDriveController::arcadeDrive(double throttle, double turn)
{
    int throttle_cmd = static_cast<int>(std::clamp(throttle, -127.0, 127.0));
    int turn_cmd = static_cast<int>(std::clamp(turn, -127.0, 127.0));
    m_chassis.arcade(throttle_cmd, turn_cmd, false);
}

void VisionDriveController::stopDrive()
{
    m_chassis.arcade(0, 0, false);
}