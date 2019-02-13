/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Util/PrefUtil.h"
#include "ctre/Phoenix.h"

/*

double calculateF() {
  const int base = PrefUtil::getSetInt("Intake.position.base", 0);
  const int feedForwardZeroPos = PrefUtil::getSetInt("Intake.position.ffzeropos", 600);   // zero position for k
  const double feedForwardZero = PrefUtil::getSet("Intake.position.ffzero", 0.11);        // ff for holding zero

  int currentPosition = Talon->GetSelectedSensorPosition(0);
  if (currentPosition < base) {
      currentPosition += 4096;
  }
  double theta = ((currentPosition - (base + feedForwardZeroPos)) / 4096.0) * TWO_PI;
  double k = feedForwardZero * cos(theta / 2);    // Account for 2:1 gearing
  frc::SmartDashboard::PutNumber("Rotate Angle", (theta * 180) / M_PI);

} */


class IntakeMagicMotionManager {
 public:
  explicit IntakeMagicMotionManager(std::shared_ptr<WPI_TalonSRX> _talon) : talon(_talon) {}

  void Init() {
    double P = PrefUtil::getSet("Intake.MM.P", 1);
    double V = PrefUtil::getSet("Intake.MM.V", 500);
    double A = PrefUtil::getSet("Intake.MM.A", 500);

    talon->ConfigMotionCruiseVelocity(V);
    talon->ConfigMotionAcceleration(A);
    talon->Config_kP(0, P);
    talon->Config_kI(0, 0);
    talon->Config_kD(0, 0);
    talon->Config_kF(0, 0);
    talon->ClearMotionProfileTrajectories();
    talon->ClearMotionProfileHasUnderrun();
  }

  void Run(double targetPoint) {
    // talon->Config_kF(CalculateF(targetPoint));
  }

  double CalculateF(double target) {
  }

 private:
  const std::shared_ptr<WPI_TalonSRX> talon;
};
