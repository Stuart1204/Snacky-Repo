#include "vision_drive.hpp"
#include "globals.hpp"
#include "elevator.cpp"
#include "piston.cpp"
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

static int getObjectX(const pros::AIVision::Object &obj) {
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object)) {
        return obj.object.element.xoffset + obj.object.element.width / 2;
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)) {
        return (obj.object.tag.x0 + obj.object.tag.x1 + obj.object.tag.x2 + obj.object.tag.x3) / 4;
    }
    return obj.object.color.xoffset + obj.object.color.width / 2;
}

static int getObjectY(const pros::AIVision::Object &obj) {
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object)){
        return obj.object.element.yoffset + obj.object.element.height / 2;
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)){
        return (obj.object.tag.y0 + obj.object.tag.y1 + obj.object.tag.y2 + obj.object.tag.y3) / 4;
    }
    return obj.object.color.yoffset + obj.object.color.height / 2;
}

static int getObjectWidth(const pros::AIVision::Object &obj) {
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object)) {
        return obj.object.element.width;
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)) {
        return std::abs(obj.object.tag.x1 - obj.object.tag.x0);
    }
    return obj.object.color.width;
}

static int getObjectHeight(const pros::AIVision::Object &obj) {
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::object)) {
        return obj.object.element.height;
    }
    if (pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)){
        return std::abs(obj.object.tag.y1 - obj.object.tag.y0);
    }
    return obj.object.color.height;
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
    constexpr int HALF_BLUE = 6;    // <-- new
    constexpr int HALF_RED = 7;     // <-- new
    constexpr int HALF_YELLOW = 8;  // <-- new
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
    m_ai_vision.enable_detection_types(pros::AivisionModeType::colors, pros::AivisionModeType::tags);
    m_ai_vision.set_tag_family(pros::AivisionTagFamily::tag_21H7);

    // --- Colors (from VEXcode robot config) ---

    pros::AIVision::Color blue = {
        .id = ColorIDs::BLUE,
        .red = 57,
        .green = 176,
        .blue = 206,
        .hue_range = 16.0,
        .saturation_range = 0.46};
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
        .hue_range = 15.0,
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

DriveToResult VisionDriveController::acquireAlliancePin(
    int x,
    int y,
    int visionSwitchDistance,
    bool pivotIntake,
    int timeout,
    bool isYellow)  // <-- new parameter
{
    m_running = true;
    m_abort_flag = false;

    uint32_t routineStart = pros::millis();
    uint32_t lastSeenMs = pros::millis();
    uint32_t intakeStartMs = 0;

    bool everFound = false;
    bool finalStopped = false;
    bool usedFallback = false;

    const double odomArrivalRadius = 2.0;
    const double lostCloseRadius = 5.0;
    const int sensorArmDelayMs = 300;

    intakeLift.setState(pivotIntake);

    // -----------------------------------------------------------------------
    // Resolve target type based on alliance + isYellow flag
    //
    // isYellow = true  → look for HALF_YELLOW_PIN  (yellow-only half-pins)
    // isYellow = false → look for alliance HALF_RED_PIN or HALF_BLUE_PIN
    // -----------------------------------------------------------------------
    GameObjectType type;
    if (isYellow)
    {
        type = GameObjectType::HALF_YELLOW_PIN;
    }
    else
    {
        if (allianceColour == 1)
            type = GameObjectType::HALF_RED_PIN;
        else
            type = GameObjectType::HALF_BLUE_PIN;
    }

    // =========================================================================
    // Phase 1 — Odometry approach to vision handoff point
    // =========================================================================
    lemlib::Pose cur = m_chassis.getPose();
    double dx = static_cast<double>(x) - cur.x;
    double dy = static_cast<double>(y) - cur.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    double stopShort = static_cast<double>(visionSwitchDistance) + m_cam_fwd_in;

    if (dist > stopShort)
    {
        double ratio = (dist - stopShort) / dist;
        double wp_x = cur.x + dx * ratio;
        double wp_y = cur.y + dy * ratio;

        int timeLeft = timeout - static_cast<int>(pros::millis() - routineStart);
        if (timeLeft <= 0)
            timeLeft = 500;

        int moveTimeout = std::min(
            timeLeft,
            static_cast<int>(dist / 20.0 * 1000.0) + 1500);

        m_chassis.moveToPoint(wp_x, wp_y, moveTimeout, {.forwards = true, .maxSpeed = static_cast<float>(m_config.approach_speed), .minSpeed = 20, .earlyExitRange = static_cast<float>(std::max(3.0, stopShort * 0.3))});
        if (m_abort_flag) {
            stopDrive();
            m_intake_motor.move(0);
            m_running = false;
            return DriveToResult::ABORTED;
        }
    }

    // =========================================================================
    // Phase 2 — Face target
    // =========================================================================
    {
        int timeLeft = timeout - static_cast<int>(pros::millis() - routineStart);
        if (timeLeft <= 0)
            timeLeft = 500;

        m_chassis.turnToPoint(x,y, std::min(timeLeft, 1200), {.forwards = true, .minSpeed = 20, .earlyExitRange=3});
    }

    if (m_abort_flag)
    {
        stopDrive();
        m_intake_motor.move(0);
        m_running = false;
        return DriveToResult::ABORTED;
    }

    // =========================================================================
    // Phase 3 — Start intake + elevator intake position
    // =========================================================================
    clamp.setState(false);
    elevator.setState(ElevatorState::MATCHLOAD);
    elevator.waitUntilComplete();
    m_intake_motor.move(127);
    intakeStartMs = pros::millis();

    // =========================================================================
    // Phase 4 — Vision centers, odom decides arrival
    // =========================================================================
    double prevError = 0.0;
    double integral = 0.0;

    while (true)
    {
        if (m_abort_flag)
        {
            stopDrive();
            m_intake_motor.move(0);
            elevator.setState(ElevatorState::STOW);
            m_running = false;
            return DriveToResult::ABORTED;
        }

        lemlib::Pose pose = m_chassis.getPose();
        double dxNow = static_cast<double>(x) - pose.x;
        double dyNow = static_cast<double>(y) - pose.y;
        double distToTarget = std::sqrt(dxNow * dxNow + dyNow * dyNow);

        if (finalStopped)
        {
            stopDrive();
            intakeLift.setState(false);
            if ((pros::millis() - intakeStartMs) > static_cast<uint32_t>(sensorArmDelayMs) &&
                colourSortSensor.get_proximity() > 100) {   
                m_intake_motor.move(127);
                pros::delay(750);
                elevator.setState(ElevatorState::PICKUP);
                m_running = false;
                if (elevator.isStateComplete()) {
                    m_intake_motor.move(0);
                }
                return DriveToResult::SUCCESS;
            }

            if (pros::millis() - routineStart > static_cast<uint32_t>(timeout)) {
                intakeLift.setState(false);
                m_intake_motor.move(127);
                pros::delay(750);
                elevator.setState(ElevatorState::PICKUP);
                m_running = false;
                if (elevator.isStateComplete()) {
                    m_intake_motor.move(0);
                }
                return usedFallback ? DriveToResult::OBJECT_LOST : DriveToResult::TIMEOUT;
            }
            pros::delay(20);
            continue;
        }

        if (distToTarget <= odomArrivalRadius) {
            finalStopped = true;
            stopDrive();
            pros::delay(20);
            continue;
        }

        auto target = findBestTarget(type);

        if (target.has_value()) {
            const auto &obj = target.value();
            everFound = true;
            lastSeenMs = pros::millis();

            double error = static_cast<double>(getObjectX(obj)) - m_config.frame_center_x;

            integral += error;
            integral = std::clamp(integral, -500.0, 500.0);

            if ((error > 0.0 && prevError < 0.0) || (error < 0.0 && prevError > 0.0)){
                integral = 0.0;
            }

            double deriv = error - prevError;
            prevError = error;
            double turn =
                m_config.lateral_kP * error +
                m_config.lateral_kI * integral +
                m_config.lateral_kD * deriv;

            turn = std::clamp(turn, -60.0, 60.0);

            double forward;
            if (distToTarget > 18.0)
                forward = m_config.approach_speed;
            else if (distToTarget > 8.0)
                forward = m_config.slow_approach_speed;
            else
                forward = m_config.creep_speed;

            if (std::abs(error) <= m_config.centering_deadband) {
                arcadeDrive(forward, 0.0);
            }
            else {
                double scale = 1.0 - std::min(std::abs(turn) / 60.0, 0.6);
                arcadeDrive(forward * scale, turn);
            }
        }
        else {
            if (everFound && distToTarget <= lostCloseRadius) {
                finalStopped = true;
                stopDrive();
                pros::delay(20);
                continue;
            }

            if (everFound && pros::millis() - lastSeenMs > static_cast<uint32_t>(m_config.lost_object_timeout_ms)) {
                usedFallback = true;
                stopDrive();

                m_chassis.turnToPoint(x,y,1000, {.forwards = true, .minSpeed = 0, .earlyExitRange = 1});
                m_chassis.moveToPoint(x,y, 3000, {.forwards = true, .maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 3});
                m_chassis.waitUntilDone();
                stopDrive();
                finalStopped = true;
                pros::delay(20);
                continue;
            }

            if (!everFound) {
                if (pros::millis() - routineStart > 1500){
                    usedFallback = true;
                    stopDrive();
                    m_chassis.turnToPoint(x,y,1000,{.forwards = true, .minSpeed = 0, .earlyExitRange = 3});
                    m_chassis.moveToPoint(x, y,1000,{.forwards = true,.maxSpeed = 90,.minSpeed = 30, .earlyExitRange = 3});
                    m_chassis.waitUntilDone();
                    stopDrive();
                    finalStopped = true;
                    pros::delay(20);
                    continue;
                }
                arcadeDrive(0.0, 20.0);
            }
            else {
                arcadeDrive(m_config.creep_speed, 0.0);
            }
        }

        if (pros::millis() - routineStart > static_cast<uint32_t>(timeout))
        {
            stopDrive();
            intakeLift.setState(false);
            m_intake_motor.move(127);
            pros::delay(750);
            elevator.setState(ElevatorState::PICKUP);
            m_running = false;
            if (elevator.isStateComplete()) {
                m_intake_motor.move(0);
            }
            return DriveToResult::TIMEOUT;
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
    bool is_color = pros::AIVision::is_type(obj, pros::AivisionDetectType::color);

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
    case GameObjectType::HALF_BLUE_PIN:
        // Single-color blue blob (other half is grey cup)
        return is_color && obj.id == ColorIDs::BLUE;
    case GameObjectType::HALF_RED_PIN:
        return is_color && obj.id == ColorIDs::RED;
    case GameObjectType::HALF_YELLOW_PIN:
        return is_color && obj.id == ColorIDs::YELLOW;
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
// DRIVETRAIN
// ============================================================================

void VisionDriveController::arcadeDrive(double throttle, double turn) {
    int throttle_cmd = static_cast<int>(std::clamp(throttle, -127.0, 127.0));
    int turn_cmd = static_cast<int>(std::clamp(turn, -127.0, 127.0));
    m_chassis.arcade(throttle_cmd, turn_cmd, false);
}
void VisionDriveController::stopDrive() {
    m_chassis.arcade(0, 0, false);
}