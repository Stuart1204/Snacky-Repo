#include "main.h"
#include "globals.hpp"
#include "motor_functions.hpp"

inline void mogoMechControls();

inline void diggerMechControls();

inline void wallStakeControls();

inline void drivetrainControls();

inline void initiateWallStakeLoading();

inline void intakeControls();

inline void wallStakeTipPositionControl();

inline bool mogoState = false;
inline bool diggerState = false;
inline bool wallStakePositionState = false;
inline bool Xstate = false;

inline int lastProximity = 255;