#include "main.h"
#include "globals.hpp"

//LB variables
inline int numState = 3;
inline int state[7] = {0, 22, 142, 148, 212, 172, 60}; //  Transport Height 
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

inline bool wallStakeLoading();
inline bool wallStakeScoring();
inline void checkColour();
inline void liftControl();
inline void nextState();
inline void scoreAlliance();
inline void colourSort();