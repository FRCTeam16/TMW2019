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
std::shared_ptr<JackScrews> Robot::jackScrews;


void Robot::RobotInit() {
	std::cout << "Robot::RobotInit => \n";
	robotMap.reset(new RobotMap());
	oi.reset(new OI());
    driveBase.reset(new DriveBase());

	jackScrews.reset(new JackScrews());
	runningScrews = false;

	intake.reset(new Intake());

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
	liftController.reset();

	runningScrews = false;
	runningLiftSequence = false;

    std::cout << "Robot::TeleopInit <=\n";
}
void Robot::TeleopPeriodic() {
    double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();

	double threshold = 0.1;	// Used as general joystick deadband default
	const bool lockWheels = oi->DL6->Pressed();

	/**********************************************************/
	// Jackscrew Test code
	/**********************************************************/
	if (oi->GPRB->RisingEdge()) {
		jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
		runningScrews = true;
	} else if (oi->GPLB->RisingEdge()) {
		jackScrews->ShiftAll(JackScrews::ShiftMode::kDrive);
		runningScrews = false;
	} 

	const bool gamepadLTPressed = oi->GetGamepadLT() > 0.75;
	if (gamepadLTPressed) {
		if (oi->GPY->RisingEdge()) {
			jackScrews->SetLiftMode(JackScrews::LiftMode::kBack);
		} else if (oi->GPA->RisingEdge()) {
			jackScrews->SetLiftMode(JackScrews::LiftMode::kFront);
		} else if (oi->GPB->RisingEdge()) {
			jackScrews->SetLiftMode(JackScrews::LiftMode::kAll);
		}
	}

	if (oi->GPStart->RisingEdge()) {
		runningLiftSequence = true;
		if (liftController.get() == nullptr) {
			liftController.reset(new LiftController());
			std::cout << "Constructed new LiftController\n";
		}
		liftController->Next();
	}

	/**********************************************************
	 * Intake 
	**********************************************************/
	if (!gamepadLTPressed) {
		if (oi->DR2->Pressed()) {
			intake->IntakeCargo();
		} else if (oi->DR1->Pressed()) {
			intake->EjectCargo();
		} else if (oi->DL2->Pressed()) {
			intake->IntakeHatch();
		} else if (oi->DL1->Pressed()) {
			intake->EjectHatch();
		} else {
			//intake->Halt();
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

	// FIXME: Crawler needs to go somewhere
	const double crawlSpeed = oi->GetGamepadLeftStick();
	if (fabs(crawlSpeed) > 0.03) {
		RobotMap::crawlMotor->Set(crawlSpeed);
	} else {
		RobotMap::crawlMotor->Set(0.0);
	}


	double start = frc::Timer::GetFPGATimestamp();
	if (speedModeTest) {
		driveBase->SetConstantVelocity(twistInput, 0.60);
		driveBase->Diagnostics();
	} else if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else if (runningLiftSequence) {
		liftController->Run();	// currently calls jackscrews -run
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


void Robot::InitSubsystems() {
    std::cout << "Robot::InitSubsystems =>\n";
	jackScrews->Init();
	visionSystem->Init();
	intake->Init();
	// status & dms currently don't have init
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
	frc::SmartDashboard::PutNumber("FL Encoder", driveBase->wheels[0]->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("FR Encoder", driveBase->wheels[1]->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("RL Encoder", driveBase->wheels[2]->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("RR Encoder", driveBase->wheels[3]->GetDriveEncoderPosition() );

	frc::SmartDashboard::PutNumber("FL Out Amps", driveBase->wheels[0]->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("FR Out Amps", driveBase->wheels[1]->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("RL Out Amps", driveBase->wheels[2]->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("RR Out Amps", driveBase->wheels[3]->GetDriveOutputCurrent() );

	frc::SmartDashboard::PutNumber("FL Vel", driveBase->wheels[0]->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("FR Vel", driveBase->wheels[1]->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("RL Vel", driveBase->wheels[2]->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("RR Vel", driveBase->wheels[3]->GetDriveVelocity() );

	DriveInfo<double> sanity = driveBase->GetDriveEncoderPositions();
	frc::SmartDashboard::PutNumber("Sanity", sanity.RL);
	
}


void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

