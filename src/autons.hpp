#include "main.h"
#include "globals.hpp"

inline void autonCheckColour();
inline bool multitaskIntakeControls = false;
inline int intakeThreadCounter = 0;

inline void multitaskIntakeControl_f(void* param);
inline lemlib::Pose currentPose = chassis.getPose();
inline bool exitLoop = false;

inline void driveDistance(int distance, int maxSpeed);

inline void driveToDistance(int distance, int maxSpeed, bool resetPos);

inline void testingAuton();

inline void lineUpWallStake(bool direction);
inline void skillsAuton();
inline void skillsAutonSecondHalf();

inline void mogoRushBlue();

inline void nonMogoRushRed();

inline void nonMogoRushBlue();

inline void elimsMogoRushRed();

inline void sigAWPBlue();

