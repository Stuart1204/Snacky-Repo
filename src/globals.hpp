#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

// -- Defined Ports -- //

//motor ports
#define lBPort -1
#define lMPort 2
#define lFPort -4

#define rBPort -13
#define rMPort 15
#define rFPort 16

#define lowIntakePort 12
#define upperIntakePort -11
#define scoringRollerPort 21

//sensor ports
#define imuPort 7
#define colourSortPort 9
#define parkSensorPort 5
#define leftDistPort 4
#define rightDistPort 15
#define frontDistPort 6

#define verticalEncoderPort -17

//solenoids
#define leftWingPort 'C'
#define rightWingPort 'A'
#define tonguePort 'F'
#define parkPort 'D'
#define intakeLiftPort 'E'
#define odomLiftPort 'B'
#define middleGoalDescorePort 'D'

//drivetrain specs
#define driveTrackWidth 11.375
#define driveWheelType lemlib::Omniwheel::NEW_325
#define driveRPM 450

// auton selection variable
inline int autonSelectionEnum;
inline int startingSide;

//global variables
inline int allianceColour;
inline int scoreHeight; //1 for long goal, 2 for middle goal
inline int blockCounter = 0;
inline bool enableColourSort = false;
inline bool slowLowGoal = false;

// global constants
const inline float FRONT_DIST_OFFSET = 3.5;
const inline float LEFT_DIST_OFFSET = 2.5;
const inline float RIGHT_DIST_OFFSET = 2.5;
const inline float MM_TO_INCHES = 0.03937007874;
const inline float CENTIDEG_TO_INCHES = 0.00017889624;
const inline float DEG_TO_RAD = (M_PI / 180.0);

// used for communicating between intake and park piston
inline bool activatePark = false;

// used for communicating between drivetrain and buttons
inline bool matchloadShake = false;

// intake states enum
enum IntakeState {
    IDLE = 0,
    INTAKE = 1,
    OUTTAKE = 2,
    SCOREHIGH = 3,
    SCOREMID = 4,
    SCORELOW = 5,
    PARK = 6,
    LOWINTAKE = 7,
    LOWOUTTAKE = 8,
    INTAKEWITHEJECT = 9,
    STOPMID = 10,
    SCOREHIGHSLOW = 11
};

// aliance colour
enum AllianceColour {
    RED = 1,
    BLUE = 2
};

// auton selector enum
enum Selection {
    SKILLS = 1,
    MATCH = 2,
    LEFT = 3,
    RIGHT = 4,
    LONG_CONTROL = 5,
    LONG_AND_CENTER = 6,
    SOLO_AWP = 7,
    NI_SAWP = 8,
    MATCHLOAD_SCORE = 9,
    NOTHING = 10
};

// -- Namespace Configurations -- //

namespace Buttons {
    constexpr auto R1 = pros::E_CONTROLLER_DIGITAL_R1;
    constexpr auto R2 = pros::E_CONTROLLER_DIGITAL_R2;
    constexpr auto L1 = pros::E_CONTROLLER_DIGITAL_L1;
    constexpr auto L2 = pros::E_CONTROLLER_DIGITAL_L2;
    constexpr auto A = pros::E_CONTROLLER_DIGITAL_A;
    constexpr auto B = pros::E_CONTROLLER_DIGITAL_B;
    constexpr auto Y = pros::E_CONTROLLER_DIGITAL_Y;
    constexpr auto X = pros::E_CONTROLLER_DIGITAL_X;
    constexpr auto UP = pros::E_CONTROLLER_DIGITAL_UP;
    constexpr auto DOWN = pros::E_CONTROLLER_DIGITAL_DOWN;
    constexpr auto LEFT = pros::E_CONTROLLER_DIGITAL_LEFT;
    constexpr auto RIGHT = pros::E_CONTROLLER_DIGITAL_RIGHT;

    // controller buttons specific
    constexpr auto wing = Buttons::X;
    constexpr auto park = Buttons::B;
    constexpr auto tongue = Buttons::A;
    constexpr auto intakeLift = Buttons::Y;
    constexpr auto colourSort = Buttons::RIGHT;
}

namespace MotorConfigs {
    constexpr auto BRAKE = pros::E_MOTOR_BRAKE_BRAKE;
    constexpr auto COAST = pros::E_MOTOR_BRAKE_COAST;
    constexpr auto HOLD = pros::E_MOTOR_BRAKE_HOLD;
    constexpr auto RED = pros::E_MOTOR_GEAR_RED;
    constexpr auto GREEN = pros::E_MOTOR_GEAR_GREEN;
    constexpr auto BLUE = pros::E_MOTOR_GEAR_BLUE;
    constexpr auto DEGREES = pros::E_MOTOR_ENCODER_DEGREES;
}



// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
inline pros::Motor lowerIntake(lowIntakePort, pros::v5::MotorGears::rpm_600);
inline pros::Motor upperIntake(upperIntakePort, pros::v5::MotorGears::rpm_600);
inline pros::Motor scoringRoller(scoringRollerPort, pros::v5::MotorGears::rpm_600);

inline pros::MotorGroup leftMotors({lBPort, lMPort, lFPort}, pros::v5::MotorGears::rpm_600);
inline pros::MotorGroup rightMotors({rBPort, rMPort, rFPort}, pros::v5::MotorGears::rpm_600);

// pneumatics
inline pros::adi::DigitalOut leftWingPiston(leftWingPort, LOW);
inline pros::adi::DigitalOut rightWingPiston(rightWingPort, LOW);
inline pros::adi::DigitalOut tonguePiston(tonguePort, LOW);
inline pros::adi::DigitalOut parkPiston(parkPort, LOW);
inline pros::adi::DigitalOut intakeLiftPiston(intakeLiftPort, LOW);
inline pros::adi::DigitalOut odomLiftPiston(odomLiftPort, LOW);
inline pros::adi::DigitalOut middleGoalDescorePiston(middleGoalDescorePort, LOW);


// Inertial Sensor on port
inline pros::Imu inertial_sensor(imuPort);
inline pros::Optical colourSortSensor(colourSortPort);
inline pros::Optical parkSensor(parkSensorPort);
inline pros::Distance leftDistSensor(leftDistPort);
inline pros::Distance rightDistSensor(rightDistPort);
inline pros::Distance frontDistSensor(frontDistPort);

inline pros::Rotation vertical_rotation(verticalEncoderPort); 
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, 2.05, 0);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              driveTrackWidth, 
                              driveWheelType, 
                              driveRPM, 
                              8 // chase power is 2. If we had traction wheels, it would have been 8
);

// Odometry sensors struct
inline lemlib::OdomSensors sensors{
	&vertical_tracking_wheel, 
	nullptr,			  
	nullptr, 
	nullptr,			  
	&inertial_sensor	  // inertial sensor
};

// lateral motion controller
inline lemlib::ControllerSettings linearController(5.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            29, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            200, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(2.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             16, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// Creating chassis object
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

#endif