/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "OI.h"

std::unique_ptr<OI> Robot::oi;
std::shared_ptr<DriveBase> Robot::driveBase;


void Robot::RobotInit() {
	std::cout << "Robot::RobotInit => \n";
	robotMap.reset(new RobotMap());
	oi.reset(new OI());
    driveBase.reset(new DriveBase());

	jackScrews.reset(new JackScrews());
	runningScrews = false;
	jackScrews->SetAllSolenoidState(JackScrews::ShiftMode::kDrive);

	visionSystem.reset(new VisionSystem());

    statusReporter.reset(new StatusReporter());
    statusReporter->Launch();
    dmsProcessManager.reset(new DmsProcessManager(statusReporter));
	std::cout << "Robot::TeleopInit <=\n";
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
}

void Robot::AutonomousInit() {
	cout << "AutonomousInit => TeleopInit\n";
	TeleopInit();
}
void Robot::AutonomousPeriodic() {
	cout << "AutonomousPeriodic => TeleopPeriodic\n";
	// if teleop call removed add frc::Scheduler::GetInstance()->Run();
	TeleopPeriodic();
}

void Robot::TeleopInit() {
    std::cout << "Robot::TeleopInit =>\n";
    InitSubsystems();
	driveBase->InitTeleop();
	assert(oi.get() != nullptr);
	runningScrews = false;
	jackScrews->SetAllSolenoidState(JackScrews::ShiftMode::kDrive);
    std::cout << "Robot::TeleopInit <=\n";
}
void Robot::TeleopPeriodic() {
    double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();
	double threshold = 0.1;
	const bool lockWheels = oi->DL6->Pressed();

	/**********************************************************/
	// Jackscrew Test code
	/**********************************************************/
	if (oi->GPRB->RisingEdge()) {
		//Jackscrew shifter->Next();
		jackScrews->SetAllSolenoidState(JackScrews::ShiftMode::kJackscrews);
		runningScrews = true;
	} else if (oi->GPLB->RisingEdge()) {
		//Jackscrew shifter->Previous();
		jackScrews->SetAllSolenoidState(JackScrews::ShiftMode::kDrive);
		runningScrews = false;
	} 

	if (runningScrews) {
		if (oi->GetGamepadRT() > 0.10) {
			jackScrews->ExtendClosedLoop(true);
		} else if (oi->GetGamepadLT() > .10) {
			jackScrews->ExtendClosedLoop(false);
		} else {
			jackScrews->Stop();
		}
	}

	/**********************************************************
	 * Vision Testing
	**********************************************************/
	const bool visionMode = oi->DL8->Pressed();	// controls drive
	if (oi->DR11->RisingEdge()) {
		visionSystem->ToggleCameraMode();
	}

	
	/**********************************************************
	 * Testing and Diagnostics
	**********************************************************/
	const bool speedModeTest = oi->DL7->Pressed();
	const bool distanceMode = oi->DL8->Pressed();
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);
	

	/**********************************************************
	 * Drive Control
	**********************************************************/
	double twistInput = oi->GetJoystickTwist(threshold);
	
	if (oi->DL4->Pressed()) {
		driveBase->SetTargetAngle(-60.0);
		twistInput = driveBase->GetCrabTwistOutput();
	} else 	if (oi->DL5->Pressed()) {
		driveBase->SetTargetAngle(60.0);
		twistInput = driveBase->GetCrabTwistOutput();
	}


	double start = frc::Timer::GetFPGATimestamp();
	if (speedModeTest) {
		driveBase->SetConstantVelocity(twistInput, 0.60);
		driveBase->Diagnostics();
	} else if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else if (runningScrews) {
		jackScrews->RunOpenLoop(-oi->GetJoystickY(threshold));
		jackScrews->Run();
	} else if (visionMode) {
		visionSystem->Run();
	} else {
		if (!lockWheels) {
			driveBase->Crab(
				twistInput,
				-oi->GetJoystickY(threshold),
				oi->GetJoystickX(threshold),
				true);
		} else {
			driveBase->Crab(0, 0, 0, true);
		}
	}
	

	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("DriveBaseRun", (now-start) * 1000);
	RunSubsystems();
	InstrumentSubsystems();

	long elapsed = (frc::Timer::GetFPGATimestamp() - startTime) * 1000.0;
	SmartDashboard::PutNumber("Teleop Period (ms)", elapsed);
}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}


void Robot::InitSubsystems() {
    std::cout << "Robot::InitSubsystems =>\n";
	Robot::jackScrews->Init();
	std::cout << "Robot::InitSubsystems <=\n";
}

void Robot::RunSubsystems() {
    double start = frc::Timer::GetFPGATimestamp();
    dmsProcessManager->Run();
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("DMS Time", (now-start) * 1000);
}

void Robot::InstrumentSubsystems() {
    Robot::jackScrews->Instrument();
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

