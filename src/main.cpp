#include "main.h"
#include "Functional/drivebase.hpp"
#include "Functional/subsystems.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "EZ-Template/api.hpp"

pros::ADIDigitalIn increase('F');
pros::ADIDigitalIn decrease('G');


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

 // Auton("Skills Run", functional::Skills),

void initialize() {
	ez::print_ez_template();

	pros::delay(500); // S from doing anything while legacy ports configure.
	
	ez::as::limit_switch_lcd_initialize(&increase, &decrease);

	ez::as::auton_selector.add_autons( {
    Auton("Solo Win Point\n Schticks Middle Goal if nobody grabbed it\n70 Points", functional::SoloAWP_MIDSnatch),
	Auton("Solo Win Point\n Solo AWP, goes back to original side after\n50 Points", functional::SoloAWP),
	Auton("Neutral Steal Platform Down \nx Points", functional::NeutralStealPlatDown),
	Auton("Neutral Steal Platform Up \nx Points", functional::NeutralStealPlatUp),
	Auton("Mid Neutral Steal Platform Up \nx Points", functional::Mid_NeutralStealPlatUp),
	Auton("Mid Neutral Steal Platform Down \nx Points", functional::Mid_NeutralStealPlatDown),
	Auton("Funny \nx Points", functional::funnyAutohehe),
	Auton("Reg Race \nx Points", functional::fwdBack),
	Auton("Skills Run", functional::Skills),
	} );
  ez::as::initialize();
}
	
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	wing.set_value(1);
	chomp.set_value(0);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {

  ez::as::auton_selector.call_selected_auton(); 
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

 bool switchs = false;
 int ct = 1;
 bool clamp = false;
void opcontrol() {

	bool tog = false;

	while (true) {

		if ( limit1.get_value() == 1)
		intake.move(0);
		else if ( tog != true ) {
		intake.move(110);
		}

		if (primary.get_digital(DIGITAL_UP))
		tog = true;
		else if ( primary.get_digital(DIGITAL_RIGHT))
		tog = false;
		if ( primary.get_digital(DIGITAL_L1) )
		pros::Task task(functional::clampOn);
		else if ( primary.get_digital(DIGITAL_L2) )
		pros::Task task(functional::clampOff);

		functional::setDrive();
		functional::setLift();
		functional::setChomp();
		functional::special();
		


		pros::delay(20);
	}
}
