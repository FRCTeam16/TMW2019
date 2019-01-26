/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/JackScrews.h"
#include <robot.h>
#include "Subsystems/Drive/DriveBase.h"

  void JackScrews::Run() {
    Preferences *prefs = Preferences::GetInstance();
    if (!prefs->ContainsKey("ProtoScrewSpeed")) {
      prefs->PutLong("ProtoScrewSpeed", 500);
    } 
    double rpm = prefs->GetLong("ProtoScrewSpeed");

    double speed = 0.0;
    if (running) {
      speed = rpm * direction;
    }

    for (auto const& wheel : Robot::driveBase->wheels) {
      wheel->UseClosedLoopDrive(speed);
    }
  }

  void JackScrews::SetAllSolenoidState(bool extend) {
    SetFrontSolenoidState(extend);
    SetRearSolenoidState(extend);
  }
  void JackScrews::SetFrontSolenoidState(bool extend) {
    frontAxleSolenoid->Set(extend);
  }
  void JackScrews::SetRearSolenoidState(bool extend) { 
    rearAxleSolenoid->Set(extend);
   }
  // void JackScrews::SetExtendFL(bool extend) {
  //   if 
  // }
  // void JackScrews::SetExtendFR(bool extend) {
  // }
  // void JackScrews::SetExtendRL(bool extend) {
  // }
  // void JackScrews::SetExtendRR(bool extend) {
  // }
  
  void JackScrews::SetExtendScrews(bool extend, bool running_) {
    direction = extend ? 1 : -1;
    running = running_;
  }