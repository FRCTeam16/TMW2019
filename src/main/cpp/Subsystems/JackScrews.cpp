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
  frontAxis.push_back(Robot::driveBase->wheels[0]);
  frontAxis.push_back(Robot::driveBase->wheels[1]);
  rearAxis.push_back(Robot::driveBase->wheels[2]);
  rearAxis.push_back(Robot::driveBase->wheels[3]);

  allWheels.reserve(frontAxis.size() + rearAxis.size());
  allWheels.insert(allWheels.end(), frontAxis.begin(), frontAxis.end());
  allWheels.insert(allWheels.end(), rearAxis.begin(), rearAxis.end());
}

void JackScrews::Run()
{
  if (!enabled) { return; }

  std::cout << "Jackscrews::Run() =>\n";
  
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

void JackScrews::RunOpenLoop(double speed) {
  targetPosition = Position::kNone;
  openLoopSpeed = speed;
}

void JackScrews::RunControlled(LiftMode liftMode, Position targetPosition_) {
  this->SetLiftMode(liftMode);
  targetPosition = targetPosition_;
  controlTimeStart = frc::Timer::GetFPGATimestamp();

  frc::Preferences *prefs = frc::Preferences::GetInstance();
  auto fl = new JackScrewCalculator(Robot::driveBase->wheels[0], prefs->GetInt("JackScrew.FL.dist"), controlTimeStart);
  auto fr = new JackScrewCalculator(Robot::driveBase->wheels[1], prefs->GetInt("JackScrew.FR.dist"), controlTimeStart);
  auto rl = new JackScrewCalculator(Robot::driveBase->wheels[2], prefs->GetInt("JackScrew.RL.dist"), controlTimeStart);
  auto rr = new JackScrewCalculator(Robot::driveBase->wheels[3], prefs->GetInt("JackScrew.RR.dist"), controlTimeStart);

  auto di = new DriveInfo<std::shared_ptr<JackScrewCalculator>>();
  di->FL.reset(fl);
  di->FR.reset(fr);
  di->RL.reset(rl);
  di->RR.reset(rr);
  calculators.reset(di);

  enabledCalculators = DriveInfo<bool>{true};
  if (LiftMode::kFront == liftMode) {
    enabledCalculators.RL = false;
    enabledCalculators.RR = false;
  } else if (LiftMode::kBack == liftMode) {
    enabledCalculators.FL = false;
    enabledCalculators.FR = false;
  } else {
    enabledCalculators.FL = true;
    enabledCalculators.FR = true;
    enabledCalculators.RL = true;
    enabledCalculators.RR = true;
  }

  std::cout << "Enabled FL: " << enabledCalculators.FL << "\n"
            << "        FR: " << enabledCalculators.FR << "\n"
            << "        RL: " << enabledCalculators.RL << "\n"
            << "        RR: " << enabledCalculators.RR << "\n";
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

void JackScrews::DoControlled() {
  std::vector<std::shared_ptr<JackScrewCalculator>> activeCalcs;
  if (enabledCalculators.FL) {
    activeCalcs.push_back(calculators->FL);
  }
  if (enabledCalculators.FR) {
    activeCalcs.push_back(calculators->FR);
  }
  if (enabledCalculators.RL) {
    activeCalcs.push_back(calculators->RL);
  }
  if (enabledCalculators.RR) {
    activeCalcs.push_back(calculators->RR);
  }
  std::cout << "Active calcs size: " << activeCalcs.size() << "\n";
  if (activeCalcs.empty()) {
    std::cout << "No active jackscrews\n";
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
    // frc::SmartDashboard::PutNumber("JS." + std::to_string(i) + ".diff", diff );
    if (diff >= kMaximumDisplacementThreshold) {
      if (currentCalc->GetLastChange() > lowest->GetLastChange()) {
        const double delta = speedDir * 0.02; // alternate do ratio?
        const double newSpeed = currentCalc->GetControlSpeed() - delta;
        currentCalc->SetControlSpeed(newSpeed);
        // frc::SmartDashboard::PutNumber("JS." + std::to_string(i) + ".speed", newSpeed );
      }
    }
    // frc::SmartDashboard::PutNumber("JS." + std::to_string(i) + ".accum", currentCalc->GetAccumulatedPosition() );
    std::cout << "calc[" << i << "] speed = " << currentCalc->GetControlSpeed() 
              << " | change = " << currentCalc->GetLastChange()
              << " | accum = " << currentCalc->GetAccumulatedPosition() 
              << "\n";
    currentCalc->Run();
    if (currentCalc->IsClosedLoop()) {
      std::cout << "Closed Loop detected, exiting loop\n";
      enabledCalculators.FL = false;
      enabledCalculators.FR = false;
      enabledCalculators.RL = false;
      enabledCalculators.RR = false;
      break;
    }
  }
  // FIXME: Hack
  if (!enabledCalculators.FL && !enabledCalculators.FR && !enabledCalculators.RL && !enabledCalculators.RR) {
    
  }
    
}