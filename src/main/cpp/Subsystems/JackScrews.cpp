/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/JackScrews.h"
#include "Robot.h"
#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "Util/RampUtil.h"
#include <iostream>
#include <algorithm>

JackScrews::JackScrews() : frontAxleSolenoid(RobotMap::frontAxleSolenoid), rearAxleSolenoid(RobotMap::rearAxleSolenoid)
{
  auto wheels = Robot::driveBase->GetWheels();
  frontAxis.push_back(wheels.FL);
  frontAxis.push_back(wheels.FR);
  rearAxis.push_back(wheels.RL);
  rearAxis.push_back(wheels.RR);

  allWheels.reserve(frontAxis.size() + rearAxis.size());
  allWheels.insert(allWheels.end(), frontAxis.begin(), frontAxis.end());
  allWheels.insert(allWheels.end(), rearAxis.begin(), rearAxis.end());

  auto di = new DriveInfo<std::shared_ptr<JackScrewControl>>();
  di->FL.reset(new JackScrewControl(wheels.FL));
  di->FR.reset(new JackScrewControl(wheels.FR));
  di->RL.reset(new JackScrewControl(wheels.RL));
  di->RR.reset(new JackScrewControl(wheels.RR));
  jackScrews.reset(di);
}

void JackScrews::Init() {
  this->ShiftAll(JackScrews::ShiftMode::kDrive);
}

void JackScrews::Run()
{
  std::cout << "jackScrews->FL: " << jackScrews->FL.get() << " : " << static_cast<int>(jackScrews->FL->GetCurrentState()) << "\n";
  bool allSwerve =
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->FL->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->FR->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->RL->GetCurrentState()) &&
    (JackScrewControl::JackScrewState::kSwerve == jackScrews->RR->GetCurrentState());
  
  if (allSwerve) {
    // Should be controlled by driveBase, not jackscrew
    std::cout << "All swerve in the house\n";
    return;
  }

  // std::cout << "Jackscrews::Run() =>\n";
  
  switch (targetPosition) {
    case Position::kNone:
      // do nothing
      break;

    case Position::kDown:
    case Position::kUp:
      std::cout << "DoControlled\n";
      DoControlled();      
      break;
  }

  jackScrews->FL->Run();
  jackScrews->FR->Run();
  jackScrews->RL->Run();
  jackScrews->RR->Run();
  
  // std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::ShiftAll(ShiftMode shiftMode) {
  ShiftFront(shiftMode);
  ShiftRear(shiftMode);
}

void JackScrews::ShiftFront(ShiftMode shiftMode) {
  frontAxleSolenoid->Set(static_cast<bool>(shiftMode));
  if (ShiftMode::kJackscrews == shiftMode) {
    std::cout << "Shifted front to open\n";
    jackScrews->FL->InitOpenLoop(0.0);
    jackScrews->FR->InitOpenLoop(0.0);
  } else {
    std::cout << "Shifted front to swerve\n";
    jackScrews->FL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
    jackScrews->FR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
  }
}

void JackScrews::ShiftRear(ShiftMode shiftMode) {
  rearAxleSolenoid->Set(static_cast<bool>(shiftMode));
   JackScrewControl::JackScrewState newState = JackScrewControl::JackScrewState::kSwerve;
  if (ShiftMode::kJackscrews == shiftMode) {
    std::cout << "Shifted rear to open\n";
    jackScrews->RL->InitOpenLoop(0.0);
    jackScrews->RR->InitOpenLoop(0.0);
  } else {
    std::cout << "Shifted rear to swerve\n";
    jackScrews->RL->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
    jackScrews->RR->SetCurrentState(JackScrewControl::JackScrewState::kSwerve);
  }
}

void JackScrews::SetLiftMode(LiftMode liftMode) {
  currentLiftMode = liftMode;
}

/**
 * Deprecated?
 */
bool JackScrews::InPosition() {
  if (targetPosition != Position::kNone) {
    return controlHoldMode; // TODO: Check screw calculators?
  }
}

void JackScrews::ConfigureOpenLoop(double speed) {
  targetPosition = Position::kNone;
  openLoopSpeed = speed;

  // Initialize jackscrews we will be controlling
  DriveInfo<bool> toInit {false};
  if (LiftMode::kFront == currentLiftMode) {
    toInit.FL = true;
    toInit.FR = true;
  } else if (LiftMode::kBack == currentLiftMode) {
    toInit.RL = true;
    toInit.RR = true;
  } else {
    toInit.FL = true;
    toInit.FR = true;
    toInit.RL = true;
    toInit.RR = true;
  }

  if (toInit.FL) {
    // std::cout << "InitOpen FL\n";
    jackScrews->FL->InitOpenLoop(speed);  
  }
  if (toInit.FR) {
    // std::cout << "InitOpen FR\n";
    jackScrews->FR->InitOpenLoop(speed);
  }
  if (toInit.RL) {
    // std::cout << "InitOpen RL\n";
    jackScrews->RL->InitOpenLoop(speed);
  }
  if (toInit.RR) {
    // std::cout << "InitOpen RR\n";
    jackScrews->RR->InitOpenLoop(speed);  ;
  }
}

void JackScrews::ConfigureControlled(LiftMode liftMode, Position targetPosition_) {
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();

  frc::Preferences *prefs = frc::Preferences::GetInstance();
  // auto wheels = Robot::driveBase->GetWheels();

  // Initialize jackscrews we will be controlling
  DriveInfo<bool> toInit {false};
  if (LiftMode::kFront == currentLiftMode) {
    toInit.FL = true;
    toInit.FR = true;
  } else if (LiftMode::kBack == currentLiftMode) {
    toInit.RL = true;
    toInit.RR = true;
  } else {
    toInit.FL = true;
    toInit.FR = true;
    toInit.RL = true;
    toInit.RR = true;
  }

  if (toInit.FL) {
    std::cout << "Init FL\n";
    jackScrews->FL->Init(prefs->GetDouble("JackScrew.FL.dist"), controlTimeStart);  
  }
  if (toInit.FR) {
    std::cout << "Init FR\n";
    jackScrews->FR->Init(prefs->GetDouble("JackScrew.FR.dist"), controlTimeStart);
  }
  if (toInit.RL) {
    std::cout << "Init RL\n";
    jackScrews->RL->Init(prefs->GetDouble("JackScrew.RL.dist"), controlTimeStart);
  }
  if (toInit.RR) {
    std::cout << "Init RR\n";
    jackScrews->RR->Init(prefs->GetDouble("JackScrew.RR.dist"), controlTimeStart);
  }
}

/**
 * Called from Run()
 */
void JackScrews::DoOpenLoop() {
  // Determine the set of wheels we are manipulating
  // std::vector<std::shared_ptr<SwerveWheel>> wheels;

  // switch(currentLiftMode) {
  //   case LiftMode::kFront:
  //     jackScrews->FL->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->FL->SetControlSpeed(openLoopSpeed);
  //     jackScrews->FR->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->FR->SetControlSpeed(openLoopSpeed);
  //     // wheels = frontAxis;
  //     break;
  //   case LiftMode::kBack:
  //     jackScrews->RL->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->RL->SetControlSpeed(openLoopSpeed);
  //     jackScrews->RR->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->RR->SetControlSpeed(openLoopSpeed);
  //     // wheels = rearAxis;
  //     break;
  //   default:
  //     // jackScrews->FL->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->FL->SetControlSpeed(openLoopSpeed);
  //     // jackScrews->FR->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->FR->SetControlSpeed(openLoopSpeed);
  //     // jackScrews->RL->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->RL->SetControlSpeed(openLoopSpeed);
  //     // jackScrews->RR->SetCurrentState(JackScrewControl::JackScrewState::kOpenLoop);
  //     jackScrews->RR->SetControlSpeed(openLoopSpeed);
  //     // wheels = allWheels;
  // }

  // for (auto const &wheel : wheels) {
  //   wheel->UseOpenLoopDrive(openLoopSpeed);
  // }
}

/**
 * Called from Run()
 */
void JackScrews::DoControlled() {
  std::vector<std::shared_ptr<JackScrewControl>> activeCalcs;
  std::vector<std::shared_ptr<JackScrewControl>> inactiveCalcs;
  if (LiftMode::kFront == currentLiftMode) {
    activeCalcs.push_back(jackScrews->FL);
    activeCalcs.push_back(jackScrews->FR);
  } else if (LiftMode::kBack == currentLiftMode) {
    activeCalcs.push_back(jackScrews->RL);
    activeCalcs.push_back(jackScrews->RR);
  } else {
    activeCalcs.push_back(jackScrews->FL);
    activeCalcs.push_back(jackScrews->FR);
    activeCalcs.push_back(jackScrews->RL);
    activeCalcs.push_back(jackScrews->RR);
  }
  std::cout << "Active calcs size: " << activeCalcs.size() << "\n";
  // Debug
  frc::SmartDashboard::PutNumber("FL Target", jackScrews->FL->GetTargetDistance());
  frc::SmartDashboard::PutNumber("FR Target", jackScrews->FR->GetTargetDistance());
  frc::SmartDashboard::PutNumber("RL Target", jackScrews->RL->GetTargetDistance());
  frc::SmartDashboard::PutNumber("RR Target", jackScrews->RR->GetTargetDistance());


  // Capture accumulated positions
  bool inHoldPosition = false;
  auto lowest = activeCalcs[0].get();
  for (int i=1; i < activeCalcs.size(); i++) {
    if (activeCalcs[i]->GetAccumulatedPosition() < lowest->GetAccumulatedPosition()) {
      lowest = activeCalcs[i].get();
      inHoldPosition |= activeCalcs[i]->IsClosedLoop();
    }
  }
  if (inHoldPosition) {
    std::cout << "Detected active jack screw control in hold position\n";
    return;
  }
  std::cout << "Configured lowest\n";

  const double MAX_SPEED = 1.0;
  const double kMinAccumulatedPosition = lowest->GetAccumulatedPosition();
  const int speedDir = static_cast<int>(targetPosition);
  if (fabs(lowest->GetControlSpeed()) != MAX_SPEED) {
    lowest->SetControlSpeed(MAX_SPEED * speedDir);
  }
  std::cout << "Set lowest control speed to: " << (MAX_SPEED * speedDir) << "\n";


  // check wheels outside of threshold, slow them down
  const double kMaximumDisplacementThreshold = 1;
  bool exitOpenLoop = false;
  for (int i=0; i<activeCalcs.size(); i++) {
    auto currentCalc = activeCalcs[i].get();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      exitOpenLoop = true;
      break;
    }

    const int diff = abs(currentCalc->GetAccumulatedPosition() - kMinAccumulatedPosition);
    if (diff >= kMaximumDisplacementThreshold) {
      if (currentCalc->GetLastChange() > lowest->GetLastChange()) {
        const double delta = speedDir * 0.02; // alternate do ratio?
        const double newSpeed = currentCalc->GetControlSpeed() - delta;
        currentCalc->SetControlSpeed(newSpeed);
      }
    }
    std::string label = "calc[" + std::to_string(i) + "] Speed";
    frc::SmartDashboard::PutNumber(label, currentCalc->GetControlSpeed());

    std::cout << "calc[" << i << "] speed = " << currentCalc->GetControlSpeed() 
              << " | change = " << currentCalc->GetLastChange()
              << " | accum = " << currentCalc->GetAccumulatedPosition() 
              << "\n";

    
  } // end active calc

  if (exitOpenLoop) {
    for (auto const& control : activeCalcs) {
      control->Hold();
    }
  }
}