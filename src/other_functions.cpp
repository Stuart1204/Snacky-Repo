#ifndef other_functions_cpp
#define other_functions_cpp

#include "globals.hpp"
#include "piston.cpp"

inline void setWings(bool state) {
    leftWing.setState(state);
    rightWing.setState(state);
}

#endif