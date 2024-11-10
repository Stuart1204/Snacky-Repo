#include "main.h"
#include "globals.hpp"

//for wall stake loading
inline bool wallStakeLoad = false;
inline bool inWallStake = false;
inline int counter = 0;

//for wall stake scoring
inline bool wallStakeDirection = false;
inline bool wallStakeScore = false;

inline bool wallStakeLoading();
inline bool wallStakeScoring();
inline void checkColour();