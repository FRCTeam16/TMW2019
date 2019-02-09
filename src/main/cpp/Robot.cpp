/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include "OI.h"
#include "Util/PrefUtil.h" 
#include "Util/UtilityFunctions.h"

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
	crawler.reset(new Crawler());

	visionSystem.reset(new VisionSystem());
    statusReporter.reset(new StatusReporter());
    // statusReporter->Launch();
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

	runningScrews = false;				// flag for when jackscrew control is manual
	runningLiftSequence = false;		// flag for when jackscrew control is automatic

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
		jackScrews->ConfigureOpenLoop(0.0);
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
	if (oi->DR1->Pressed()) {
		// intake->IntakeCargo();
		double bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.bottomSpeed", 1.0);
		double topBeaterSpeed = PrefUtil::getSet("Intake.IntakeCargo.topSpeed", 1.0);
		intake->SetBottomBeaterSpeed(bottomBeaterSpeed);
		intake->SetTopBeaterSpeed(topBeaterSpeed);
	} else if (oi->DR2->Pressed()) {
		// intake->EjectCargo();
		double bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.bottomSpeed", -1.0);
		double topBeaterSpeed = PrefUtil::getSet("Intake.EjectCargo.topSpeed", -1.0);
		intake->SetBottomBeaterSpeed(bottomBeaterSpeed);
		intake->SetTopBeaterSpeed(topBeaterSpeed);
	} else if (oi->DL1->Pressed()) {
		// intake->IntakeHatch();
		double bottomBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.bottomSpeed", -1.0);
		double topBeaterSpeed = PrefUtil::getSet("Intake.IntakeHatch.topSpeed", 0.0);
		intake->SetBottomBeaterSpeed(bottomBeaterSpeed);
		intake->SetTopBeaterSpeed(topBeaterSpeed);
	} else if (oi->DL2->Pressed()) {
		// intake->EjectHatchManual();
		double bottomBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.bottomSpeed", -1.0);
		double topBeaterSpeed = PrefUtil::getSet("Intake.EjectHatch.topSpeed", 0.0);
		intake->SetBottomBeaterSpeed(bottomBeaterSpeed);
		intake->SetTopBeaterSpeed(topBeaterSpeed);
	} else {
		intake->Stop();
	}

	const bool gamepadRTPressed = oi->GetGamepadRT() > 0.75;
	if (!gamepadLTPressed) {
		if (gamepadRTPressed) {
			if (oi->GPY->RisingEdge()) {
				std::cout << "Running ejector manually\n";
				solenoidState.ejector = !solenoidState.ejector;
				intake->SetEjectorState(solenoidState.ejector);
			} else if (oi->GPA->RisingEdge()) {
				solenoidState.hatchChatch = !solenoidState.hatchChatch;
				intake->SetHatchState(solenoidState.hatchChatch);
			} else if (oi->GPX->RisingEdge()) {
				solenoidState.gripper = !solenoidState.gripper;
				intake->SetGripperState(solenoidState.gripper);
			}
		} else {
			// if (oi->GPY->RisingEdge()) {
			// 	intake->SetIntakePosition(Intake::IntakePosition::kCargoShot);
			// } else if (oi->GPA->RisingEdge()) {
			// 	intake->SetIntakePosition(Intake::IntakePosition::kLevelOne);
			// } else if (oi->GPX->RisingEdge()) {
			// 	intake->SetIntakePosition(Intake::IntakePosition::kStarting);
			// } else if (oi->GPB->RisingEdge()) {
			// 	intake->SetIntakePosition(Intake::IntakePosition::kFloor);
			// }
			if (oi->GPY->RisingEdge()) {
				intake->EjectHatch();
			} else if (oi->GPA->RisingEdge()) {
				intake->IntakeHatch();
			}
		}
	}

	const double leftStickAmt = oi->GetGamepadLeftStick();
	if (fabs(leftStickAmt) > threshold) {
		intake->SetPositionSpeed(leftStickAmt, true);
	} else {
		// will not trigger switch to open loop mode
		intake->SetPositionSpeed(0.0, false);
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
//	const bool distanceMode = oi->DL8->Pressed();
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);


	/**********************************************************
	 * Drive Control
	**********************************************************/
	double twistInput = oi->GetJoystickTwist(threshold);

	if (visionMode) {
		driveBase->SetTargetAngle(calculateLockAngle(RobotMap::gyro->GetYaw()));
	}

		
	// Crawler control
	if (oi->GetGamepadDPad() == OI::DPad::kDown) {
		crawler->Back();
	} else if (oi->GetGamepadDPad() == OI::DPad::kUp) {
		crawler->Forward();
	} else {
		crawler->Stop();
	}


	double start = frc::Timer::GetFPGATimestamp();
	if (speedModeTest) {
		driveBase->SetConstantVelocity(twistInput, 0.60);
		driveBase->Diagnostics();
	} else if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else if (runningLiftSequence) {
		liftController->Run();
	} else if (runningScrews) {
		// manual control
		jackScrews->ConfigureOpenLoop(-oi->GetJoystickY(threshold));
		jackScrews->Run();
	} else {
		if (!lockWheels) {

			double xMove = oi->GetJoystickX();
			bool useGyro = true;
			if (visionMode) { 
				xMove = visionSystem->GetLastDriveInfo()->xspeed;
				useGyro = false;
			}
			driveBase->Crab(
				twistInput,
				-oi->GetJoystickY(threshold),
				xMove,
				useGyro);
		} else {
			driveBase->Crab(0, 0, 0, true);
		}
	}
	

	double now = frc::Timer::GetFPGATimestamp();
	double driveBaseTime = (now-start) * 1000;
	SmartDashboard::PutNumber("DriveBaseRun", (now-start) * 1000);
	RunSubsystems();
	InstrumentSubsystems();

	double elapsed = (frc::Timer::GetFPGATimestamp() - startTime) * 1000.0;
	SmartDashboard::PutNumber("Teleop Period (ms)", elapsed);
	SmartDashboard::PutNumber("Non-DriveBase Time (ms)", (elapsed - driveBaseTime));
}	


void Robot::InitSubsystems() {
    std::cout << "Robot::InitSubsystems =>\n";
	jackScrews->Init();
	visionSystem->Init();
	intake->Init();
	crawler->Init();
	// status & dms currently don't have init
	std::cout << "Robot::InitSubsystems <=\n";
}

void Robot::RunSubsystems() {
    double start = frc::Timer::GetFPGATimestamp();
    dmsProcessManager->Run();
	intake->Run();
	crawler->Run();
	visionSystem->Run(); 
	// liftController takes over driving so is in teleop loop
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("Subsystem Times", (now-start) * 1000);
}

void Robot::InstrumentSubsystems() {
	auto wheels = driveBase->GetWheels();
	frc::SmartDashboard::PutNumber("FL Encoder", wheels.FL->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("FR Encoder", wheels.FR->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("RL Encoder", wheels.RL->GetDriveEncoderPosition() );
	frc::SmartDashboard::PutNumber("RR Encoder", wheels.RR->GetDriveEncoderPosition() );

	frc::SmartDashboard::PutNumber("FL Out Amps", wheels.FL->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("FR Out Amps", wheels.FR->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("RL Out Amps", wheels.RL->GetDriveOutputCurrent() );
	frc::SmartDashboard::PutNumber("RR Out Amps", wheels.RR->GetDriveOutputCurrent() );

	frc::SmartDashboard::PutNumber("FL Vel", wheels.FL->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("FR Vel", wheels.FR->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("RL Vel", wheels.RL->GetDriveVelocity() );
	frc::SmartDashboard::PutNumber("RR Vel", wheels.RR->GetDriveVelocity() );

	jackScrews->Instrument();
	intake->Instrument();
	crawler->Instrument();
}


void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

