#include "EZ-Template/util.hpp"
#include "drivebase.hpp"
#include "main.h"
#include "EZ-Template/api.hpp"
#include "pneumatics.hpp"
#include "pros/motors.h"

// Easily Edit the motor port V
// Negative Values are reversed

const int FRONTLEFT_PORT = -15;
const int MIDLEFT_PORT = -4;
const int TOPLEFT_PORT = 3;

const int FRONTRIGHT_PORT = 20;
const int MIDRIGHT_PORT = 1;
const int TOPRIGHT_PORT = -2;

pros::Motor frontLeft(15, true);
pros::Motor midLeft(4, true);
pros::Motor topLeft(3, false);

pros::Motor frontRight(20, false);
pros::Motor midRight(1, false);
pros::Motor topRight(2, true);

void tank(double pwr, int a) {

  if ( a == 0 ) {
  frontLeft.move(pwr);
  midLeft.move(pwr);
  topLeft.move(pwr);

  frontRight.move(pwr);
  midRight.move(pwr);
  topRight.move(pwr);
  }
  else if ( a == 1 ) {
  frontLeft.move(-pwr);
  midLeft.move(-pwr);
  topLeft.move(-pwr);

  frontRight.move(pwr);
  midRight.move(pwr);
  topRight.move(pwr);
  }
  else if ( a == 2 ) {
  frontLeft.move(-pwr);
  midLeft.move(-pwr);
  topLeft.move(-pwr);

  frontRight.move(0);
  midRight.move(0);
  topRight.move(0);
  }
  else if ( a == 3 ) {
  frontLeft.move(0);
  midLeft.move(0);
  topLeft.move(0);

  frontRight.move(pwr);
  midRight.move(pwr);
  topRight.move(pwr);
  }

}

double maxlinvel, maxlinaccel, maxlinjerk;

Drive drivebase (
  // Left Chassis Ports (negative port will reverse it!)
  //  The first port is the sensored port (when trackers are not used!)
  {FRONTLEFT_PORT, MIDLEFT_PORT, TOPLEFT_PORT}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{FRONTRIGHT_PORT, MIDRIGHT_PORT, TOPRIGHT_PORT}

  // IMU Port
  ,5

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  ,3.25

  // Cartridge RPM
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,1.7
);

namespace functional {

       void initDrive() {

              // Curves the Joystick Values in order to have better driving accuracy.
              drivebase.set_curve_default(3);

              // Applys a constant P loop that powers the motor to hold its position ever so slightly.
              drivebase.set_active_brake(0.06);

              // Sets PID constants
              setMogoConstants(0);

              // Sets Slew constants
              drivebase.set_slew_min_power(70, 60);
              drivebase.set_slew_distance(2, 2);


              // Sets MP limits
              maxlinvel = 1.0;
              maxlinaccel = 2.0;
              maxlinjerk = 10.0;


       }
       void setMogoConstants(int mogos) {

              switch ( mogos ) {

                     case 0:
                     drivebase.set_pid_constants(&drivebase.forward_drivePID, .25, 4, 1, 5);
                     drivebase.set_pid_constants(&drivebase.backward_drivePID, .25, 4, 1, 5);
                     drivebase.set_pid_constants(&drivebase.headingPID, 11, 0, 20, 0);
                     drivebase.set_pid_constants(&drivebase.turnPID, 3, .8, 2, 1.6);
                     drivebase.set_pid_constants(&drivebase.swingPID, 3, .8, 2, 1.6);
                     break;

                     case 1:
                     drivebase.set_pid_constants(&drivebase.forward_drivePID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.backward_drivePID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.headingPID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.turnPID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.swingPID, 0, 0, 0, 0);
                     break;
                     
                     case 2:
                     drivebase.set_pid_constants(&drivebase.forward_drivePID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.backward_drivePID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.headingPID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.turnPID, 0, 0, 0, 0);
                     drivebase.set_pid_constants(&drivebase.swingPID, 0, 0, 0, 0);
                     break;

              }
       }
       void setDrive() {
              // Setting Arcade Control for the robot drive ( Single Joystick )
              drivebase.arcade_standard(ez::SINGLE);
       }
       void changeBrake(double kP) {
              // Little Helper Function Renamed under the functional namespace to get rid of confusion
              drivebase.set_active_brake(kP);
       }

       /**
        * END DRIVE FUNCTIONS
        * START AUTON FUNCTIONS
        */

        /**
        * Assumed Constants
        */
       const int DRIVE_SPEED = 110; 
       const int TURN_SPEED  = 90;
       const int SWING_SPEED = 90;

       /**
 * @brief 
 * Helper Functions
 */
 
///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater then the slew distance + a few inches


  drivebase.set_drive_pid(24, DRIVE_SPEED, true);
  drivebase.wait_drive();

  drivebase.set_drive_pid(-12, DRIVE_SPEED);
  drivebase.wait_drive();

  drivebase.set_drive_pid(-12, DRIVE_SPEED);
  drivebase.wait_drive();
}



///
// Turn Example
///
void turn_example() {
  // The first parameter is target degrees
  // The second parameter is max speed the robot will drive at


  drivebase.set_turn_pid(90, TURN_SPEED);
  drivebase.wait_drive();

  drivebase.set_turn_pid(45, TURN_SPEED);
  drivebase.wait_drive();

  drivebase.set_turn_pid(0, TURN_SPEED);
  drivebase.wait_drive();
}

void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is target degrees
  // The third parameter is speed of the moving side of the drive


  drivebase.set_swing_pid(ez::LEFT_SWING, 45, SWING_SPEED);
  drivebase.wait_drive();

  drivebase.set_drive_pid(24, DRIVE_SPEED, true);
  drivebase.wait_until(12);

  drivebase.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
  drivebase.wait_drive();

}
void combining_movements() {
  drivebase.set_drive_pid(24, DRIVE_SPEED, true);
  drivebase.wait_drive();

  drivebase.set_turn_pid(45, TURN_SPEED);
  drivebase.wait_drive();

  drivebase.set_swing_pid(ez::RIGHT_SWING, -45, TURN_SPEED);
  drivebase.wait_drive();

  drivebase.set_turn_pid(0, TURN_SPEED);
  drivebase.wait_drive();

  drivebase.set_drive_pid(-24, DRIVE_SPEED, true);
  drivebase.wait_drive();
}

       /**
        * @brief 
       * Going for both win points by yourself if your teammate doesnt have an auton and its an easy match
       */
       void clampTask() {
         pros::delay(450);
         bclamp.set_value(1);
	       pros::delay(150);
	       tilt.set_value(1);
       }
       void yaya() {
         lift.move(60);
         pros::delay(500);
         lift.move(0);
       }
       void SoloAWP_MIDSnatch() {
       /// Initalize Everything
         drivebase.reset_drive_sensor();
         setMogoConstants(0);
         lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
         chomp.set_value(0);
         /// Back Up into first mogo & Swing to yellow angle
         pros::delay(20);
         drivebase.set_drive_pid(-5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         pros::Task clampy(clampOn);

         drivebase.set_drive_pid(5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         drivebase.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(20, DRIVE_SPEED);
         drivebase.wait_drive();

         lift.move(60);
         intake.move(110);

         drivebase.set_swing_pid(ez::LEFT_SWING, 180, SWING_SPEED);
         drivebase.wait_drive();

         lift.move(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(93, 50);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-10, 50);
         drivebase.wait_drive();

         lift.move(-120);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(215, TURN_SPEED);
         pros::delay(550);

         lift.move(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(10, DRIVE_SPEED);
         drivebase.wait_drive();

         chomp.set_value(1);
         pros::Task yeye(yaya);

         wing.set_value(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-40, DRIVE_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(145, TURN_SPEED);
         drivebase.wait_until(15);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-75, DRIVE_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(15, DRIVE_SPEED);
         

         wing.set_value(1);
         
         clampOff();

       }
       void SoloAWP() {
       /// Initalize Everything
         drivebase.reset_drive_sensor();
         setMogoConstants(0);
         lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
         chomp.set_value(0);
         /// Back Up into first mogo & Swing to yellow angle
         pros::delay(20);
         drivebase.set_drive_pid(-5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         pros::Task clampy(clampOn);

         drivebase.set_drive_pid(5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         drivebase.set_swing_pid(ez::LEFT_SWING, 90, SWING_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(20, DRIVE_SPEED);
         drivebase.wait_drive();

         lift.move(60);
         intake.move(110);

         drivebase.set_swing_pid(ez::LEFT_SWING, 180, SWING_SPEED);
         drivebase.wait_drive();

         lift.move(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(93, 50);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-10, 50);
         drivebase.wait_drive();

         lift.move(-120);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(215, TURN_SPEED);
         pros::delay(550);

         lift.move(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(10, DRIVE_SPEED);
         drivebase.wait_drive();

         chomp.set_value(1);
         pros::Task yeye(yaya);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(90,TURN_SPEED);
         drivebase.reset_drive_sensor();

         drivebase.set_drive_pid(-10, DRIVE_SPEED);
         drivebase.wait_drive();

       }

       void fwdBack() {
         tank(127,0);
         pros::delay(750);
         chomp.set_value(1);
         pros::delay(100);
         tank(-80,0);
         pros::delay(200);
         tank(-127,0);
         pros::delay(700);
         tank(0,0);
       }
       /**
        * @brief 
        * Mogo-Side down steal the Neutral goal and load rings
        */
       void NeutralStealPlatDown() {
         wing.set_value(0);
         drivebase.reset_drive_sensor();

         drivebase.set_drive_pid(-40, 200);
         drivebase.wait_until(-24);
         drivebase.set_turn_pid(-150, 200);
         drivebase.wait_drive();

         drivebase.set_turn_pid(-110, 200);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-10, DRIVE_SPEED);
         drivebase.wait_drive();

         pros::delay(1000);

         drivebase.set_turn_pid(-200, 60);
         drivebase.wait_drive();

         wing.set_value(1);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-15, DRIVE_SPEED);
         drivebase.wait_drive();

         clampOn();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(5, DRIVE_SPEED);
         drivebase.wait_drive();

         drivebase.set_turn_pid(-165,TURN_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-50, DRIVE_SPEED);
         drivebase.wait_drive();

         clampOff();

         lift.move(-40);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(10, DRIVE_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();




         
       }
       /**
       * @brief 
       * Mogo-Side up steal the Neutral goal and load rings
       */
       void NeutralStealPlatUp() {
         wing.set_value(0);
         drivebase.reset_drive_sensor();

         drivebase.set_drive_pid(-40, 200);
         drivebase.wait_until(-23);
         drivebase.set_turn_pid(-145, 200);
         drivebase.wait_drive();
         drivebase.set_turn_pid(-125, 200);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-40, 65);
         drivebase.wait_drive();

         pros::Task task1(clampOn);

         pros::delay(1000);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(20, DRIVE_SPEED);
         drivebase.wait_drive();

         wing.set_value(1);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(-70, TURN_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(30, 65);
         drivebase.wait_drive();

         chomp.set_value(1);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-9, 127);
         drivebase.wait_drive();

       }
       /**
       * @brief 
       * Mogo-Side Platoform up steal the CENTER Neutral goal and load rings
       */
       void Mid_NeutralStealPlatUp() {

       }
       /**
       * @brief 
       * Mogo-Side Platoform down steal the CENTER Neutral goal and load rings
       */
       void Mid_NeutralStealPlatDown() {
        wing.set_value(0);
        drivebase.reset_drive_sensor();

        drivebase.set_drive_pid(-42, 200);
        drivebase.wait_until(5);
        
        wing.set_value(1);

        drivebase.set_drive_pid(60, 200);


       }
       /**
       * @brief 
       * Mogo-Side Platoform down steal the CENTER Neutral goal and load rings
       */
       void funnyAutohehe() {

       }

       void jjclampone() {
         pros::delay(500);
         chomp.set_value(1);
         pros::delay(100);
         lift.move(100);
         pros::delay(550);
         lift.move(0);
       }

       void jclampone() {
         pros::delay(300);
         chomp.set_value(1);
       }
         
       
       void tallunclamp() {

         pros::delay(900);
         chomp.set_value(0);
         pros::delay(200);
         lift.move(-60);
         pros::delay(300);
         lift.move(0);


       }

       void downe() {
         pros::delay(200);
         lift.move(-100);
       }
       /**
        * @brief 
       * Skills Routine hopeing for 300+ pts
        */
       void Skills() {
         /// Initalize Everything
         drivebase.reset_drive_sensor();
         setMogoConstants(0);
         chomp.set_value(0);
         /// Back Up into first mogo & Swing to yellow angle
         pros::delay(20);
         drivebase.set_drive_pid(-5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         pros::Task clampy(clampOn);

         drivebase.set_drive_pid(5, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         drivebase.set_swing_pid(ez::LEFT_SWING, 107, SWING_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         /// Drive forward and clamp on yellow mogo

         drivebase.set_drive_pid(40, DRIVE_SPEED, true, true);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         chomp.set_value(1);

         /// Turn to ring angle, lift arm get rings

         lift.move(50);
         lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

         drivebase.set_drive_pid(30, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         intake.move(100);

         drivebase.set_swing_pid(ez::LEFT_SWING, 180, SWING_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         lift.move(70);

         drivebase.set_drive_pid(20, DRIVE_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();
         

         /// Turn towards platform and place on platform & Backup

         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         lift.move(-80);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(15, DRIVE_SPEED);
         drivebase.wait_drive();

         lift.move(0);
         chomp.set_value(0);

         pros::delay(100);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-16, DRIVE_SPEED);
         drivebase.wait_drive();

         lift.move(-90);

         drivebase.set_turn_pid(-90, TURN_SPEED);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         /// Drop back mogo and pick up tall yellow

         pros::Task clampOffa(clampOff);
         intake.move(0);
         lift.move(0);
         pros::delay(550);

         pros::Task jojojojo(jjclampone);

         drivebase.set_drive_pid(44, DRIVE_SPEED, true, true);
         drivebase.wait_drive();
         drivebase.reset_drive_sensor();

         /// Move yellow and back up to blue angle

         drivebase.set_turn_pid(12, 60);
         drivebase.wait_drive();

         /// Drop tall mogo & back up into blue mogo

         pros::Task tallywooo(tallunclamp);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-55, 60);
         drivebase.wait_drive();

         /// Pickup blue mogo and drive forward & turn to yellow angle

         pros::Task clampy2(clampOn);
         pros::delay(100);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(19, DRIVE_SPEED);
         drivebase.wait_drive();

         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();

         /// Drive forward and grab yellow mogo & swing to platform

         pros::Task yellowGrab(jjclampone);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(40, DRIVE_SPEED, true, true);
         drivebase.wait_drive();

         lift.move(90);
         intake.move(110);

         drivebase.reset_drive_sensor();
         drivebase.set_swing_pid(ez::RIGHT_SWING, 0, SWING_SPEED);
         drivebase.wait_drive();

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(3, DRIVE_SPEED);
         drivebase.wait_drive();
         
         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();

         /// place mogo on platform & turn to old red mogo and place on platform

         lift.move(-100);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(15, DRIVE_SPEED, true, true);
         drivebase.wait_drive();

         lift.move(0);
         chomp.set_value(0);

         pros::delay(100);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-4, DRIVE_SPEED);
         drivebase.wait_drive();

         lift.move(-100);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(0, TURN_SPEED);
         drivebase.wait_drive();

         lift.move(0);
         intake.move(110);

         pros::Task oldRed(jclampone);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(30, DRIVE_SPEED, true, true);
         drivebase.wait_drive();

         lift.move(120);
         pros::delay(2000);
         pros::Task downy(downe);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();

         chomp.set_value(0);
         lift.move(0);

         pros::delay(200);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-3, DRIVE_SPEED);

         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(0, TURN_SPEED);
         drivebase.wait_drive();

         lift.move(-60);

         /// drop back mogo

         pros::Task down(clampOff);
         pros::delay(450);

         /// back into red mogo and pick it up

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(30, DRIVE_SPEED, true, true);
         drivebase.wait_drive();
         
         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(180, TURN_SPEED);
         drivebase.wait_drive();

         lift.move(0);
         
         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-20, DRIVE_SPEED, true, true);
         drivebase.wait_drive();

         pros::Task clampy3(clampOn);
         pros::delay(200);

         /// drive forward into red mogo and place on platform

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(50, DRIVE_SPEED, true, true);
         drivebase.wait_drive();

         chomp.set_value(1);
         lift.move(120);
         pros::delay(1800);
         lift.move(-50);
         
         drivebase.reset_drive_sensor();
         drivebase.set_turn_pid(90, TURN_SPEED);
         drivebase.wait_drive();

         chomp.set_value(0);
         lift.move(0);

         drivebase.reset_drive_sensor();
         drivebase.set_drive_pid(-80, DRIVE_SPEED,true, true);




         /// drive fowrad and turn to red mogo angle and drive forward and grab it

         /// backup turn towards end wall, drive foward over purple rings

         /// swing into platform and park
       }
       /*
        * @brief 
        * Not sure what to use this for yet, but its a secret auton ofc
        */
       void Fake() {

       }


}







