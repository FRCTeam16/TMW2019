/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/JackScrews.h"
#include <robot.h>
#include "Subsystems/Drive/DriveBase.h"
#include "frc/smartdashboard/SmartDashboard.h"

  void JackScrews::Run() {
    std::cout << "Jackscrews::Run(running = " << running << ") =>\n";
    Preferences *prefs = Preferences::GetInstance();
    if (!prefs->ContainsKey("ProtoScrewSpeed")) {
      prefs->PutLong("ProtoScrewSpeed", 500);
    } 
    double rpm = prefs->GetLong("ProtoScrewSpeed");

    if (running) {
      // All wheels
      if (LiftMode::kAll == currentLiftMode) {
          for (auto const& wheel : Robot::driveBase->wheels) {
          std::cout << "Open Loop Speed: " << openLoopSpeed << "\n";
          wheel->UseOpenLoopDrive(openLoopSpeed);
         }
      } else {
          // Front Axis only
          Robot::driveBase->wheels[0]->UseOpenLoopDrive(openLoopSpeed);
          Robot::driveBase->wheels[1]->UseOpenLoopDrive(openLoopSpeed);
      }
    }
    
    frc::SmartDashboard::PutNumber("FL Velocity", Robot::driveBase->wheels[0]->GetDriveVelocity());
    frc::SmartDashboard::PutNumber("FL Output Current", Robot::driveBase->wheels[0]->GetDriveOutputCurrent());

    std::cout << "Jackscrews::Run <=\n\n";
  }

  void JackScrews::SetAllSolenoidState(bool extend) {
    SetFrontSolenoidState(extend);
    SetRearSolenoidState(extend);
  }
  void JackScrews::SetFrontSolenoidState(bool extend) {
    frontAxleSolenoid->Set(extend);
    if (!extend) {
      running = false;
    }
  }
  void JackScrews::SetRearSolenoidState(bool extend) { 
    rearAxleSolenoid->Set(extend);
    if (!extend) {
      running = false;
    }
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


  void JackScrews::SetLiftMode(LiftMode liftMode) {
    currentLiftMode = liftMode;
  }

  void JackScrews::RunOpenLoop(double speed) {
    openLoopSpeed = speed;
    running = true;
  }