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

inline void homing();

inline bool mogoState = true;
inline bool diggerState = false;
inline bool wallStakePositionState = false;
inline bool Xstate = false;
inline bool wallStakeGoDown = false;

inline int prevT = 0;
inline int prevD = 0;
inline int counter = 0;
inline int interval = 60;
inline int lastProximity = 255;