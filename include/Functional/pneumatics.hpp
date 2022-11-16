#pragma once

#include "EZ-Template/api.hpp"
#include "pneumatics.hpp"
#include "pros/adi.hpp"

extern pros::ADIDigitalOut chomp;
extern pros::ADIDigitalOut bclamp;
extern pros::ADIDigitalOut tilt;
extern pros::ADIDigitalOut wing;

namespace functional {

    void setChomp();
    void setWing();

    void clampOn();
    void clampOff();

}