#include "main.h"
#include "globals.hpp"

//LB variables
inline int numState = 3;
inline int state[8] = {0, 20, 160, 148, 235, 172, 60, 200}; //  Transport Height 
inline int target = 0;


//for wall stake loading
inline bool wallStakeLoad = false;
inline bool inWallStake = false;
inline int counter = 0;

inline int ejectCounter = 0;
inline bool colourSortingEjectionState = false;

//for wall stake scoring
inline bool firstTimeSwitch = false;
inline double intakeTarget = 0;
inline bool wallStakeDirection = false;
inline bool wallStakeScore = false;

inline int lastCurrentState = 0;

// intake variables
inline int intakeJamCounter = 0;
inline int lastIntakeSpeed = 0;
inline bool intakeJustStarted = true;;

inline void checkColour();
inline void liftControl();
inline void doubleLBLoad();
inline void intakeControl();
inline bool intakeJamCheck();
inline void nextState();
inline void updateHookForLB();
inline void scoreAlliance();
inline void colourSort();