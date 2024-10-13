#include "main.h"
#include "lemlib/api.hpp"
#include "pros/adi.h"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

//motor ports
#define lBPort -7
#define lMPort -8
#define lFPort -10

#define rBPort 2
#define rMPort 3
#define rFPort 4

#define sIntakePort 1
#define bIntakePort 9
#define wallStakePort 6

//sensor ports
#define imuPort 5

#define mogoMechPort 'A'
#define horizontalEncoderPort1 'C'
#define horizontalEncoderPort2 'D'

//drivetrain specs
#define driveTrackWidth 11.775
#define driveWheelType lemlib::Omniwheel::NEW_275
#define driveRPM 450

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
inline pros::Motor wallStake(6, pros::v5::MotorGears::rpm_200);

// drive motor groups
inline pros::MotorGroup leftMotors({lBPort, lMPort, lFPort}, pros::v5::MotorGears::rpm_600);	// left motor group
inline pros::MotorGroup rightMotors({rBPort, rMPort, rFPort}, pros::v5::MotorGears::rpm_600); // right motor group
inline pros::MotorGroup intake({sIntakePort, bIntakePort});

//solenoids
inline pros::adi::DigitalOut mogoMech(mogoMechPort, LOW);

// Inertial Sensor on port
inline pros::Imu inertial_sensor(imuPort);

// back tracking wheel encoder
inline pros::adi::Encoder horizontal_encoder(horizontalEncoderPort1, horizontalEncoderPort2, false); // ports C and D, not reversed

// back tracking wheel
inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2.75, -3.1); // 2.75" wheel diameter, - offset from tracking center

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
	nullptr, 
	nullptr,			  
	&horizontal_tracking_wheel, 
	nullptr,			  
	&inertial_sensor	  // inertial sensor
};

// lateral motion controller
inline lemlib::ControllerSettings linearController(11, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            7, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            5 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(3.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             30, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// Creating chassis object
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);

#endif