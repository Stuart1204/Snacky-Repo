#include "main.h"
#include "globals.hpp"

//LB variables
inline int numState = 3;
inline int state[6] = {0, 22, 130, 148, 220, 172};
inline int target = 0;


//for wall stake loading
inline bool wallStakeLoad = false;
inline bool inWallStake = false;
inline int counter = 0;

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