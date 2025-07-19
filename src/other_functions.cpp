#ifndef other_functions_cpp
#define other_functions_cpp

#include "globals.hpp"
#include "piston.cpp"
#include "pros/optical.hpp"

inline int checkColour(pros::Optical opticalSensor) {
	if (opticalSensor.get_hue() > 160 && opticalSensor.get_hue() < 320 && opticalSensor.get_proximity() > 160) {
		return AllianceColour::BLUE; //blue
	}
	else if ((opticalSensor.get_hue() < 30 || opticalSensor.get_hue() > 330) && opticalSensor.get_proximity() > 160){
		return AllianceColour::RED; //red
	} else {
        return 0; //background
    }
}

inline void setWings(bool state) {
    leftWing.setState(state);
    rightWing.setState(state);
}

inline bool detectedState = false;
inline void countBlocks(pros::Optical sensor) {

	if(sensor.get_proximity() > 250 && !detectedState) {
		detectedState = true;
		blockCounter++;
	}
	if(sensor.get_proximity() < 250) {
		detectedState = false;
	}
}

#endif