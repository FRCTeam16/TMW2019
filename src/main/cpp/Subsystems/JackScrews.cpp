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
      break;
  }
  
  std::cout << "Jackscrews::Run <=\n\n";
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
  calculators.reset(new DriveInfo<JackScrewCalculator> {
    JackScrewCalculator(Robot::driveBase->wheels[0], prefs->GetInt("JackScrew.FL.base"), controlTimeStart),
    JackScrewCalculator(Robot::driveBase->wheels[1], prefs->GetInt("JackScrew.FR.base"), controlTimeStart),
    JackScrewCalculator(Robot::driveBase->wheels[2], prefs->GetInt("JackScrew.RL.base"), controlTimeStart),
    JackScrewCalculator(Robot::driveBase->wheels[3], prefs->GetInt("JackScrew.RR.base"), controlTimeStart)
  });

  enabledCalculators = DriveInfo<bool>{true};
  if (LiftMode::kFront == liftMode) {
    enabledCalculators.RL = false;
    enabledCalculators.RR = false;
  } else if (LiftMode::kBack == liftMode) {
    enabledCalculators.FL = false;
    enabledCalculators.FR = false;
  }
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
  std::vector<JackScrewCalculator> activeCalcs;
  if (enabledCalculators.FL) {
    activeCalcs.push_back(calculators->FL);
  } else if (enabledCalculators.FR) {
    activeCalcs.push_back(calculators->FR);
  } else if (enabledCalculators.RL) {
    activeCalcs.push_back(calculators->RL);
  } else if (enabledCalculators.RR) {
    activeCalcs.push_back(calculators->RR);
  }

  // Capture accumulated positions
  const double maximumDisplacementThreshold = 50;
  std::vector<int> accumulatedPositions;
  std::transform(activeCalcs.begin(), activeCalcs.end(), std::back_inserter(accumulatedPositions),
    [](JackScrewCalculator &c) -> int { c.GetAccumulatedPosition(); });
  int minAccumulatedPositions = *std::min_element(std::begin(accumulatedPositions), std::end(accumulatedPositions));

  auto slowest = *std::find_if(activeCalcs.begin(), activeCalcs.end(),
    [minAccumulatedPositions](JackScrewCalculator &c) -> bool { c.GetAccumulatedPosition() == minAccumulatedPositions; });

  bool isSlowestMaxed = slowest.GetControlSpeed() == 1.0;

  // May not need this condition
  if (isSlowestMaxed) {
    // check wheels outside of threshold, slow them down
    for (int i=0; i<activeCalcs.size(); i++) {
      // TODO: Check calc not in position control already
      JackScrewCalculator currentCalc = activeCalcs[i];
      const int diff = abs(accumulatedPositions[i] - minAccumulatedPositions);
      if (diff >= maximumDisplacementThreshold) {
        if (currentCalc.GetLastChange() > slowest.GetLastChange()) {
          currentCalc.SetControlSpeed(currentCalc.GetControlSpeed() - 0.01);  // alternate do ratio?
        }
      }
    }
    // If any of the motors were out of the threshold we will bump slowest here?
  } else {
    // Slowest is not maxed
    slowest.SetControlSpeed(1.0); // FIXME: Direction, make sure in threshold
  }
}