#ifndef other_functions_cpp
#define other_functions_cpp

#include "globals.hpp"
#include "piston.cpp"

inline int checkColour(pros::Optical opticalSensor) {
	if (opticalSensor.get_hue() > 160 && opticalSensor.get_hue() < 320 && opticalSensor.get_proximity() > 160) {
		return AllianceColour::BLUE; //blue
	}
	else if (opticalSensor.get_hue() < 10 && opticalSensor.get_proximity() > 160){
		return AllianceColour::RED; //red
	} else {
        return 0; //background
    }
}

inline void setWings(bool state) {
    leftWing.setState(state);
    rightWing.setState(state);
}

#endif