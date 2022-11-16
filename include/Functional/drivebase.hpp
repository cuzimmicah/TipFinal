#pragma once

#include "EZ-Template/api.hpp"

extern Drive drivebase;

namespace functional {

       void initDrive();
       void setMogoConstants(int mogos);
       void setDrive();
       void changeBrake(double kP);

       void fwdBack();

       /**
        * @brief 
       * Going for both win points by yourself if your teammate doesnt have an auton and its an easy match
       */
       void SoloAWP_MIDSnatch();
       /**
        * @brief 
       * Going for both win points by yourself if your teammate doesnt have an auton and its an easy match
       */
       void SoloAWP();
       /**
        * @brief 
        * Mogo-Side down steal the Neutral goal and load rings
        */
       void NeutralStealPlatDown();
       /**
       * @brief 
       * Mogo-Side up steal the Neutral goal and load rings
       */
       void NeutralStealPlatUp();
       /**
       * @brief 
       * Mogo-Side Platoform up steal the CENTER Neutral goal and load rings
       */
       void Mid_NeutralStealPlatUp();
       /**
       * @brief 
       * Mogo-Side Platoform down steal the CENTER Neutral goal and load rings
       */
       void Mid_NeutralStealPlatDown();
       /**
        * @brief 
       * Skills Routine hopeing for 300+ pts
        */
       void Skills();
       /**
        * @brief 
        * Not sure what to use this for yet, but its a secret auton ofc
        */
       void Fake();

       void funnyAutohehe();

}

/**
 * @brief 
 * Helper Functions
 */
 
///
// Drive Example
///
void drive_example();

///
// Turn Example
///
void turn_example();

///
// Swing Example
///
void swing_example();
void combining_movements();
