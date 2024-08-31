#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.h"
#include "pros/motors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

//motor ports
#define lBPort -18
#define lMPort -16
#define lFPort -15

#define rBPort 12
#define rMPort 13
#define rFPort 14

#define intakePort 11
#define hookPort 17
#define wallStakePort 21

//solenoid ports
#define mogoMechPort 'G'
#define diggerMechPort 'F'

//sensor ports
#define imuPort 4
#define ringTrackerPort 8
#define wallStakeAlignerPort 1

#define verticalEncoderPort 5
#define horizontalEncoderPort 9

//drivetrain specs
#define driveTrackWidth 11.775
#define driveWheelType lemlib::Omniwheel::NEW_275
#define driveRPM 450

// global variables
inline bool loadStake;
inline bool holdPosition;
inline bool positionSetting;

inline bool scoreStake;
inline bool armDirection = true;

inline bool ringInWallStake = false;


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
inline pros::Motor intake(intakePort, pros::v5::MotorGears::rpm_600);
inline pros::Motor hook(hookPort, pros::v5::MotorGears::rpm_600);
inline pros::Motor wallStake(wallStakePort, pros::v5::MotorGears::rpm_600);

// solenoids
inline pros::adi::DigitalOut mogoMech(mogoMechPort, LOW);
inline pros::adi::DigitalOut diggerMech(diggerMechPort, LOW);

// drive motor groups
inline pros::MotorGroup leftMotors({lBPort, lMPort, lFPort}, pros::v5::MotorGears::rpm_600);	// left motor group
inline pros::MotorGroup rightMotors({rBPort, rMPort, rFPort}, pros::v5::MotorGears::rpm_600); // right motor group

// Inertial Sensor on port
inline pros::Imu inertial_sensor(imuPort); // port 4

// optical sensor to track ring at the top
inline pros::Optical ringTracker(ringTrackerPort);
// distance sensor to align with wall stake
inline pros::Distance wallStakeAligner(wallStakeAlignerPort);
// inline pros::Optical wallStakeAligner(wallStakeAlignerPort);

// rotation sensorxs
inline pros::Rotation verticalEncoder(-verticalEncoderPort);
inline pros::Rotation horizontalEncoder(horizontalEncoderPort);

// back tracking wheel
inline lemlib::TrackingWheel vertical_tracking_wheel(&verticalEncoder, 2.125, 0.65); // 2.75" wheel diameter, - offset from tracking center
inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEncoder, 2.125, -3.75);
// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              driveTrackWidth, // 10 inch track width
                              driveWheelType, // using new 3.25" omnis
                              driveRPM, // drivetrain rpm is 360
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
inline lemlib::ControllerSettings linearController(8, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            33, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(3.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
                                             3, // anti windup
                                             3, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// Creating chassis object
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

#endif