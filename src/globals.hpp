#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"

#ifndef CONFIG_HPP
#define CONFIG_HPP

//motor ports
#define lBPort -9
#define lMPort -10
#define lFPort -20


#define rBPort 12
#define rMPort 3
#define rFPort 4


#define intakePort 19
#define wallStakePort 6


//sensor ports
#define imuPort 2
#define colourSortingSensorPort 13
#define lineSensorPort 'F'
#define distanceToWallPort 7
//solenoids
#define mogoMechPort 'B'
#define doinkerPort 'A'
#define ringDescorePuncherPort 'C'
#define descoreLifterPort 'D'
#define verticalEncoderPort -5
#define LB_rotationPort 14


//drivetrain specs
#define driveTrackWidth 11.375
#define driveWheelType lemlib::Omniwheel::NEW_325
#define driveRPM 480

//global variables
inline int colourSortingSelector;
inline int ringColour;
inline bool mogoState = false;

inline bool exitLoadWallStake = false;
inline int autonSelection = 0;

inline bool enableColourSort = true;

inline bool scoreAllianceState = false;

inline bool descoreState = false;

inline bool lbTip = false;

inline int currentState = 0;
inline int intakeSpeed = 0;

inline bool doubleLBLoading = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
inline pros::Motor wallStake(wallStakePort, pros::v5::MotorGears::rpm_200);
inline pros::Motor intake(intakePort, pros::v5::MotorGears::rpm_600);
inline pros::Motor lBDriveMotor(lBPort, pros::v5::MotorGears::rpm_600, pros::v5::MotorEncoderUnits::degrees);

// drive motor groups
inline pros::MotorGroup leftMotors({lBPort, lMPort, lFPort}, pros::v5::MotorGears::rpm_600);	// left motor group
inline pros::MotorGroup rightMotors({rBPort, rMPort, rFPort}, pros::v5::MotorGears::rpm_600); // right motor group

//solenoids
inline pros::adi::DigitalOut mogoMech(mogoMechPort, LOW);
inline pros::adi::DigitalOut doinker(doinkerPort, LOW);
inline pros::adi::DigitalOut descorePuncher(ringDescorePuncherPort, LOW);
inline pros::adi::DigitalOut descoreLifter(descoreLifterPort, LOW);

// Inertial Sensor on port
inline pros::Imu inertial_sensor(imuPort);
inline pros::Optical colourSortingSensor(colourSortingSensorPort);
inline pros::adi::AnalogIn lineTracker(lineSensorPort);
inline pros::Distance distanceToWall(distanceToWallPort);

// back tracking wheel encoder
//inline pros::Rotation horizontal_rotation(horizontalEncoderPort); 
inline pros::Rotation vertical_rotation(verticalEncoderPort); 
inline pros::Rotation LB_rotation(LB_rotationPort);
// back tracking wheel
//inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, 2.125, 2.3); // 2.75" wheel diameter, - offset from tracking center
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, 2.125, 1.4); // 2.75" wheel diameter, - offset from tracking center


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
inline lemlib::ControllerSettings linearController(6, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            30, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            200, // large error range timeout, in milliseconds
                                            30 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(1.6, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             12, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             40 // maximum acceleration (slew)
);

// lateral motion controller
inline lemlib::ControllerSettings mogoLinearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            40, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            200, // large error range timeout, in milliseconds
                                            30 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings mogoAngularController(1.7, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             13, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             5, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
                                             40 // maximum acceleration (slew)
);

// Creating chassis object
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors);
inline lemlib::Chassis mogoChassis(drivetrain, mogoLinearController, mogoAngularController, sensors);

#endif