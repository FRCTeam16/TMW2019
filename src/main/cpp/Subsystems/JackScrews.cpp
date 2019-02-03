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
}

void JackScrews::Init() {
  this->ShiftAll(JackScrews::ShiftMode::kDrive);
}

void JackScrews::Run()
{
  if (!enabled) { return; }

  // std::cout << "Jackscrews::Run() =>\n";
  
  switch (targetPosition) {
    case Position::kNone:
      DoOpenLoop();
      break;

    case Position::kDown:
      DoControlled();      
      break;

    case Position::kUp:
      DoControlled();      
      break;
  }
  
  // std::cout << "Jackscrews::Run <=\n\n";
}

void JackScrews::ShiftAll(ShiftMode shiftMode) {
  ShiftFront(shiftMode);
  ShiftRear(shiftMode);
  enabled = static_cast<bool>(shiftMode);
}

void JackScrews::ShiftFront(ShiftMode shiftMode) {
  frontAxleSolenoid->Set(static_cast<bool>(shiftMode));
}

void JackScrews::ShiftRear(ShiftMode shiftMode) {
  rearAxleSolenoid->Set(static_cast<bool>(shiftMode));
}

void JackScrews::SetLiftMode(LiftMode liftMode) {
  currentLiftMode = liftMode;
}

bool JackScrews::InPosition() {
  if (targetPosition != Position::kNone) {
    return controlHoldMode; // TODO: Check screw calculators?
  }
}

void JackScrews::ConfigureOpenLoop(double speed) {
  targetPosition = Position::kNone;
  openLoopSpeed = speed;
}

void JackScrews::ConfigureControlled(LiftMode liftMode, Position targetPosition_) {
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();

  frc::Preferences *prefs = frc::Preferences::GetInstance();
  auto wheels = Robot::driveBase->GetWheels();
  auto fl = new JackScrewCalculator(wheels.FL, prefs->GetInt("JackScrew.FL.dist"), controlTimeStart);
  auto fr = new JackScrewCalculator(wheels.FR, prefs->GetInt("JackScrew.FR.dist"), controlTimeStart);
  auto rl = new JackScrewCalculator(wheels.RL, prefs->GetInt("JackScrew.RL.dist"), controlTimeStart);
  auto rr = new JackScrewCalculator(wheels.RR, prefs->GetInt("JackScrew.RR.dist"), controlTimeStart);

  auto di = new DriveInfo<std::shared_ptr<JackScrewCalculator>>();
  di->FL.reset(fl);
  di->FR.reset(fr);
  di->RL.reset(rl);
  di->RR.reset(rr);
  calculators.reset(di);

  controlHoldMode = false;  // Allow open loop to position
}

/**
 * Called from Run()
 */
void JackScrews::DoOpenLoop() {
  // Determine the set of wheels we are manipulating
  std::vector<std::shared_ptr<SwerveWheel>> wheels;
  switch(currentLiftMode) {
    case LiftMode::kFront:
      wheels = frontAxis;
      break;
    case LiftMode::kBack:
      wheels = rearAxis;
      break;
    default:
      wheels = allWheels;
  }

  for (auto const &wheel : wheels) {
    wheel->UseOpenLoopDrive(openLoopSpeed);
  }
}

/**
 * Called from Run()
 */
void JackScrews::DoControlled() {
  std::vector<std::shared_ptr<JackScrewCalculator>> activeCalcs;
  if (LiftMode::kFront == currentLiftMode) {
    activeCalcs.push_back(calculators->FL);
    activeCalcs.push_back(calculators->FR);
  } else if (LiftMode::kBack == currentLiftMode) {
    activeCalcs.push_back(calculators->RL);
    activeCalcs.push_back(calculators->RR);
  } else {
    activeCalcs.push_back(calculators->FL);
    activeCalcs.push_back(calculators->FR);
    activeCalcs.push_back(calculators->RL);
    activeCalcs.push_back(calculators->RR);
  }
  std::cout << "Active calcs size: " << activeCalcs.size() << "\n";

  if (controlHoldMode) {
    std::cout << "JackScrews::DoControlled Holding Position\n";
    for (auto const &calc : activeCalcs) {
      calc->Hold();
      // Debug
      frc::SmartDashboard::PutNumber("FL Target", calculators->FL->GetTargetDistance());
      frc::SmartDashboard::PutNumber("FR Target", calculators->FR->GetTargetDistance());
      frc::SmartDashboard::PutNumber("RL Target", calculators->RL->GetTargetDistance());
      frc::SmartDashboard::PutNumber("RR Target", calculators->RR->GetTargetDistance());
    }
    return;
  }
  
  // Capture accumulated positions
  auto lowest = activeCalcs[0].get();
  for (int i=1; i < activeCalcs.size(); i++) {
    if (activeCalcs[i]->GetAccumulatedPosition() < lowest->GetAccumulatedPosition()) {
      lowest = activeCalcs[i].get();
    }
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

  for (int i=0; i<activeCalcs.size(); i++) {
    // TODO: Check calc not in position control already? currently handled in calc
    auto currentCalc = activeCalcs[i].get();
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

    currentCalc->Run();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      controlHoldMode = true;
      break;
    }
  } 
}