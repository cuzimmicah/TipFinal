#include "main.h"
#include "Functional/api.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

const char CHOMP_PORT('A');
const char BCLAMP_PORT('C');
const char TILT_PORT('B');
const char WING_PORT('E');

pros::ADIDigitalOut chomp(CHOMP_PORT, LOW);
pros::ADIDigitalOut bclamp(BCLAMP_PORT);
pros::ADIDigitalOut tilt(TILT_PORT, LOW);
pros::ADIDigitalOut wing(WING_PORT, HIGH);


namespace functional {

    void setChomp() {

        if ( primary.get_digital(pros::E_CONTROLLER_DIGITAL_A) ){
         chomp.set_value(1);
        }
        else if( primary.get_digital(pros::E_CONTROLLER_DIGITAL_B) ){
         chomp.set_value(0);
        }
    }
     
        

    void setWing() {
        if ( primary.get_digital(pros::E_CONTROLLER_DIGITAL_X) ){
         wing.set_value(1);
        }
        else if( primary.get_digital(pros::E_CONTROLLER_DIGITAL_Y) ){
         wing.set_value(0);
        }
    }

    void clampOn() {

	bclamp.set_value(1);
	pros::delay(150);
	tilt.set_value(1);
	 
    }
    void clampOff() {

	tilt.set_value(0);
	pros::delay(150);
	bclamp.set_value(0);

    }


}