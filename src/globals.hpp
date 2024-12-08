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
#define lBPort -7
#define lMPort -8
#define lFPort -10

#define rBPort 2
#define rMPort 3
#define rFPort 4

#define intakePort1 1
#define intakePort2 9
#define wallStakePort 6

//sensor ports
#define imuPort 5
#define colourSortingSensorPort 13
#define lineSensorPort 'C'
#define distanceSortPort 20
//solenoids
#define mogoMechPort 'A'
#define doinkerPort 'B'
#define ringDescorePuncherPort 'D'
#define descoreLifterPort 'E'
#define horizontalEncoderPort 17
#define verticalEncoderPort 18

//drivetrain specs
#define driveTrackWidth 11.775
#define driveWheelType lemlib::Omniwheel::NEW_275
#define driveRPM 450

//global variables
inline int colourSortingSelector;
inline bool ringColour;
inline bool mogoState = false;

inline bool exitLoadWallStake = false;
inline int autonSelection = 0;

inline bool descoreState = false;


/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motors
inline pros::Motor wallStake(6, pros::v5::MotorGears::rpm_200);
inline pros::Motor bIntake(intakePort2, pros::v5::MotorGears::rpm_600);

// drive motor groups
inline pros::MotorGroup leftMotors({lBPort, lMPort, lFPort}, pros::v5::MotorGears::rpm_600);	// left motor group
inline pros::MotorGroup rightMotors({rBPort, rMPort, rFPort}, pros::v5::MotorGears::rpm_600); // right motor group
inline pros::MotorGroup intake({intakePort1, intakePort2});

//solenoids
inline pros::adi::DigitalOut mogoMech(mogoMechPort, LOW);
inline pros::adi::DigitalOut doinker(doinkerPort, HIGH);
inline pros::adi::DigitalOut descorePuncher(ringDescorePuncherPort, LOW);
inline pros::adi::DigitalOut descoreLifter(descoreLifterPort, LOW);

// Inertial Sensor on port
inline pros::Imu inertial_sensor(imuPort);
inline pros::Optical colourSortingSensor(colourSortingSensorPort);
inline pros::adi::AnalogIn lineTracker(lineSensorPort);
inline pros::Distance distanceSort(distanceSortPort);

// back tracking wheel encoder
inline pros::Rotation horizontal_rotation(horizontalEncoderPort); 
inline pros::Rotation vertical_rotation(-verticalEncoderPort); 

// back tracking wheel
inline lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation, 2.125, 2.3); // 2.75" wheel diameter, - offset from tracking center
inline lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation, 2.75, 1.5); // 2.75" wheel diameter, - offset from tracking center


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
	&horizontal_tracking_wheel, 
	nullptr,			  
	&inertial_sensor	  // inertial sensor
};

// lateral motion controller
inline lemlib::ControllerSettings linearController(11, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            35, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            200, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(3.5, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             31, // derivative gain (kD)
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