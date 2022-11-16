#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motors.h"


pros::Motor intake(9);
pros::Motor lift(10);
pros::Controller primary(master);
pros::ADIButton limit1('D');

struct buttonStates {
    int X, A;
} bState;

namespace functional {


    void setIntake(int cual) {

        switch ( cual ) {

            case 0 : intake.move(120);
            break;
            case 1 : intake.move(0);
            break;

        }


    }

    void special() {
        if ( primary.get_digital(DIGITAL_LEFT))
        intake.move(-127);
    }

    void setLift() {
        lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

        if (primary.get_digital(pros::E_CONTROLLER_DIGITAL_R1) )
        lift.move(127);
        else if( primary.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && limit1.get_value() == 0 )
        lift.move(-127);
        else
        lift.move_velocity(0);
    }

}

