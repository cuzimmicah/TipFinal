#pragma once
#include "main.h"

extern pros::Controller primary;
extern pros::Motor intake;
extern pros::ADIButton limit1;
extern pros::Motor lift;


namespace functional {

    void setIntake(int cual);
    void special();
    void setLift();
    
}