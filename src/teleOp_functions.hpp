#include "main.h"
#include "globals.hpp"
#include "motor_functions.cpp"

inline bool doinkerState = false;
inline bool stealState = false;
inline bool climbState = false;
inline bool allianceState = false;

inline int descoreCounter = 0;

inline int counterEject = 0;
inline bool colourSortingEjectionState = false;
inline bool colourSortButtonState = true;

inline int lbTipCounter = 0;

inline void mogoMechControls();

inline void doinkerControls();

inline void descoreControls();

inline void drivetrainControls();

inline void intakeControls();

inline void wallStakeControls();

inline void climbControls();

inline void colourSortButton();

inline void newWallStake();

inline void mogoStealControls();

inline void allianceStakeControls();


