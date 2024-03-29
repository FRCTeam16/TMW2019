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
#include "Subsystems/IntakeRotate.h"
#include "Lift/LiftController.h"
#include "Subsystems/Crawler.h"
#include "Lift/LiftDrive.h"
#include "Lift/JackScrewTest.h"
#include "Subsystems/Elevator.h"
#include "Autonomous/World.h"
#include "Autonomous/AutoManager.h"

#include "AHRS.h"


struct ManualSolenoidState {
  bool ejector = false;
  bool hatchChatch = false;
  bool gripper = false;
};

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  
  void DisabledInit() override;
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
  static std::unique_ptr<Intake> intake;
  static std::shared_ptr<IntakeRotate> intakeRotate;
  static std::shared_ptr<Elevator> elevator;
  static std::unique_ptr<VisionSystem> visionSystem;
  static std::unique_ptr<Crawler> crawler;


private:
  void InitSubsystems();
	void RunSubsystems();
	void InstrumentSubsystems();
  void HandleGlobalInputs();

  std::unique_ptr<RobotMap> robotMap;
  std::shared_ptr<StatusReporter> statusReporter;
  std::unique_ptr<DmsProcessManager> dmsProcessManager;
  std::unique_ptr<AutoManager> autoManager;
  std::shared_ptr<World> world;
  bool autoInitialized = false;
  
  // teleop control variables
  bool runningScrews = false;	// true when running jackscrews
  bool runningLiftSequence = false; // true when running lift sequence
  bool dpadRightToggled = false;
  bool drPadToggled = false;
  bool dpadUpToggled = false;

  bool runInstrumentation = false;  // whether to run subsystem instrumentation

  std::unique_ptr<LiftController> liftController;

  ManualSolenoidState solenoidState;

  LiftDrive liftDrive;
  JackScrewTest jackScrewTest;

  bool initialized = false;

  std::unique_ptr<AHRS> ahrs;

};
