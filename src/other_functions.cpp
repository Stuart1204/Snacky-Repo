#ifndef other_functions_cpp
#define other_functions_cpp

#include "globals.hpp"
#include "piston.cpp"
#include "pros/optical.hpp"

inline int checkColour(pros::Optical opticalSensor) {
	if (opticalSensor.get_hue() > 160 && opticalSensor.get_hue() < 320 && opticalSensor.get_proximity() > 110) {
		return AllianceColour::BLUE; //blue
	}
	else if (opticalSensor.get_hue() > 40 && opticalSensor.get_hue() < 150 && opticalSensor.get_proximity() > 110) {
		return AllianceColour::YELLOW; //yellow
	}
	else if ((opticalSensor.get_hue() < 30 || opticalSensor.get_hue() > 330) && opticalSensor.get_proximity() > 110){
		return AllianceColour::RED; //red
	} else {
        return 0; //background
    }
}

#endif