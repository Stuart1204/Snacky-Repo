#include "main.h"
#include "globals.hpp"

inline void autonCheckColour();
inline bool multitaskIntakeControls = false;
inline int intakeThreadCounter = 0;

inline void multitaskIntakeControl_f(void* param);
inline lemlib::Pose currentPose = chassis.getPose();
inline bool exitLoop = false;

inline void driveDistance(double distance, double maxSpeed);

inline void driveToDistance(double distance, int maxSpeed, bool resetPos);

inline void testingAuton();

inline void lineUpWallStake(bool direction);
inline void autoLoadLB();
inline void autoScoreLB();
inline void checkJam();
inline void skillsAuton();
inline void skillsAutonSecondHalf();

inline void mainPosBlue();

inline void nonMogoRushRed();

inline void nonMogoRushBlue();

inline void mainMogoRushRed();

inline void elimsMogoRushRed();

inline void qualsMogoRushRed();

inline void sigAWPBlue();

inline void mainNegRed();

inline void elimsNegRed();

inline void qualsNegRed();