/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <iostream>

#include <frc/TimedRobot.h>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "OI.h"
#include "DMS/DmsProcessManager.h"
#include "DMS/StatusReporter.h"
#include "Subsystems/JackScrews.h"
#include "Subsystems/Vision/VisionSystem.h"
#include "Subsystems/Intake.h"



class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  
  void DisabledPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  // Subsystems
  static std::shared_ptr<DriveBase> driveBase; 
  static std::unique_ptr<OI> oi;
  static std::shared_ptr<JackScrews> jackScrews;

private:
  void InitSubsystems();
	void RunSubsystems();
	void InstrumentSubsystems();

  std::unique_ptr<RobotMap> robotMap;
  std::shared_ptr<StatusReporter> statusReporter;
  std::unique_ptr<DmsProcessManager> dmsProcessManager;
  std::unique_ptr<VisionSystem> visionSystem;
  std::unique_ptr<Intake> intake;

  bool runningScrews = false;	// true when running jackscrews

};
