#include "motor_functions.hpp"
#include "pros/motors.h"


bool wallStakeLoading() {
    if(intake.get_actual_velocity() < 3 && counter > 20) {
        inWallStake = true;
    } else {
        counter++;
    }

    if(inWallStake) { 
        intake.move(0);
        inWallStake = false;
        counter = 0;
        return false;
    }

    if(!inWallStake) { 
        intake.move(127);
        if(wallStake.get_position() < 245) {
            wallStake.move(30);
        } else {
            wallStake.move(2);
        }
        return true;
    }
}

bool wallStakeScoring() {
    if(wallStakeDirection) {
        if(wallStake.get_position() < 1450) {
            wallStake.move(127);
            if(wallStake.get_position() < 600) {
                intake.move(-15);
            } else {
                intake.move(0);
            }
        } else {
            wallStakeDirection = false;
        }
        return true;
    } else {
        if(wallStake.get_position() > 20) {
            wallStake.move(-127);
            return true;
        } else {
            wallStake.move(0);
            return false;
        }
    }

}