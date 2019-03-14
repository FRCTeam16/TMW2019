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
std::unique_ptr<Intake> Robot::intake;
std::shared_ptr<IntakeRotate> Robot::intakeRotate;
std::shared_ptr<Elevator> Robot::elevator;
std::unique_ptr<VisionSystem> Robot::visionSystem;


void Robot::RobotInit() {
	std::cout << "Robot::RobotInit => \n";
	robotMap.reset(new RobotMap());
	oi.reset(new OI());
    driveBase.reset(new DriveBase());

	jackScrews.reset(new JackScrews());
	runningScrews = false;

	intake.reset(new Intake());
	intakeRotate.reset(new IntakeRotate());
	intakeRotate->CalibrateHome();

	crawler.reset(new Crawler());

	elevator.reset(new Elevator());

	visionSystem.reset(new VisionSystem());
    statusReporter.reset(new StatusReporter());
    // statusReporter->Launch();
    dmsProcessManager.reset(new DmsProcessManager(statusReporter));

	autoManager.reset(new AutoManager());
	RobotMap::gyro->ZeroYaw();

	ahrs.reset(new AHRS(SPI::Port::kMXP));
	ahrs->ZeroYaw();

	std::cout << "Robot::TeleopInit <=\n";
}

void Robot::DisabledInit() {
	intakeRotate->DisabledHoldCurrentPosition();
	elevator->DisabledZeroOutput();
	visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::DriverCamera);
	// initialized = false;
}

void Robot::DisabledPeriodic() {
	frc::Scheduler::GetInstance()->Run();
	intakeRotate->DisabledHoldCurrentPosition();
	elevator->DisabledZeroOutput();
	InstrumentSubsystems();
	HandleGlobalInputs();
}

void Robot::AutonomousInit() {
	cout << "AutonomousInit\n";
	RobotMap::gyro->ZeroYaw();
	world.reset(new World());
	autoManager->Init(world);
	autoInitialized = true;
	// autoInitialized = false;			// flag for when autonomous routines are running
	InitSubsystems();
	driveBase->InitTeleop();
	liftController.reset();
	runningScrews = false;				// flag for when jackscrew control is manual
	runningLiftSequence = false;		// flag for when jackscrew control is automatic
	initialized = true;
}
void Robot::AutonomousPeriodic() {
	// cout << "AutonomousPeriodic => TeleopPeriodic\n";
	// if teleop call removed add frc::Scheduler::GetInstance()->Run();
	TeleopPeriodic();
}

void Robot::TeleopInit() {
    std::cout << "Robot::TeleopInit => initialized? " << initialized << "\n";
	if (!initialized) {
		InitSubsystems();
		driveBase->InitTeleop();
		liftController.reset();

		runningScrews = false;				// flag for when jackscrew control is manual
		runningLiftSequence = false;		// flag for when jackscrew control is automatic

		initialized = true;
		autoInitialized = false;
	} else {
		std::cout << " --- already initialized, ignoring\n";
	}
    

    std::cout << "Robot::TeleopInit <=\n";
}
void Robot::TeleopPeriodic() {
    double startTime = frc::Timer::GetFPGATimestamp();
	frc::Scheduler::GetInstance()->Run();
	
	double threshold = 0.1;	// Used as general joystick deadband default
	// const bool lockWheels = oi->DL6->Pressed();  Removed for auto testing
	const bool lockWheels = false;

	/**********************************************************/
	// Jackscrew Manual Control
	/**********************************************************/
	if (oi->DR11->RisingEdge()) {
		jackScrews->ShiftAll(JackScrews::ShiftMode::kJackscrews);
		jackScrews->ConfigureOpenLoop(0.0);
		runningLiftSequence = false;
		liftController.reset();
		runningScrews = true;
	} else if (oi->DR12->RisingEdge()) {
		jackScrews->ShiftAll(JackScrews::ShiftMode::kDrive);
		runningScrews = false;
		runningLiftSequence = false;
		liftController.reset();
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

	if (oi->GPStart->RisingEdge() && gamepadLTPressed) { 
		runningLiftSequence = true;
		if (liftController.get() == nullptr) {
			liftController.reset(new LiftController()); 
			std::cout << "Constructed new LiftController\n";
		}
		liftController->Next();
	}
	// TOOD: move runningLiftSequence check as liftController->IsRunning()

	/**********************************************************
	 * Intake 
	**********************************************************/
	// DR 3 is vision handled elsewhere
	if (oi->DR1->Pressed()) {
		intake->EjectCargo();	// score cargo
	} else if (oi->DR2->Pressed()) {
		intake->IntakeCargo();
	} else if (oi->DL4->Pressed()) {
		intake->HatchIntakeFromGround();
	} else if (oi->DL5->Pressed()) {
		intake->HatchBeaterEject();
	} else {
		intake->Stop();
	}

	if (oi->DL1->RisingEdge()) {
		intake->EjectHatch();	// score hatch
	} else if (oi->DL3->RisingEdge()) {
		intake->IntakeHatch();	// from wall
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
			// no shift modifiers
			if (oi->GPY->RisingEdge()) {
				intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kLevelOne);
			} else if (oi->GPB->RisingEdge()) {
				intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kCargoPickup);
			} else if (oi->GPX->RisingEdge()) {
				elevator->ToggleCargoShotMode();
			} else if (oi->GPA->RisingEdge()) {
				intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kFloor);
			} else if (oi->GPBack->RisingEdge()) {
				intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kCargoShot);
			}
			
		}
	}



	if (oi->GetGamepadDPad() == OI::DPad::kRight) {
		if (!dpadRightToggled) {
			intake->ToggleEjectorState();
			intake->SetGripperState(false);
			dpadRightToggled = true;
		}
	} else {
		dpadRightToggled = false;
	}

	const double kIntakeRotateThreshold = 0.07;
	const double leftStickAmt = oi->GetGamepadLeftStick();
	if (fabs(leftStickAmt) > kIntakeRotateThreshold) {
		intakeRotate->SetPositionSpeed(leftStickAmt, true);
	} else {
		// will not trigger switch to open loop mode
		intakeRotate->SetPositionSpeed(0.0, false);
	}

	/**********************************************************
	 * Elevator
	**********************************************************/
	const double rightStickAmt = oi->GetGamepadRightStick();
	if (fabs(rightStickAmt) > threshold) {
		elevator->SetOpenLoopPercent(rightStickAmt);
	} else {
		elevator->HoldPosition();
	}

/*
	const OI::DPad drHat = oi->GetDRHat();
	if (OI::DPad::kUp == drHat) {
		if (!drPadToggled) {
			elevator->IncreaseElevatorPosition();
			drPadToggled = true;
		}
	} else if (OI::DPad::kDown == drHat) {
		if (!drPadToggled) {
			elevator->DecreaseElevatorPosition();
			drPadToggled = true;
		}
	} else {
		drPadToggled = false;
	}
*/

	if (oi->GPRB->RisingEdge()) {
		elevator->IncreaseElevatorPosition();
	} else if (oi->GPLB->RisingEdge()) {
		elevator->DecreaseElevatorPosition();
	}
	

	/**********************************************************
	 * Vision
	**********************************************************/
	const bool visionMode = oi->DR3->Pressed();	// controls drive
	if (!autoInitialized) {
		if (visionMode) {
			visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::ImageProcessing);
		} else {
			visionSystem->GetLimelight()->SetCameraMode(Limelight::CameraMode::DriverCamera);
		}
	}
	HandleGlobalInputs();


	
	/**********************************************************
	 * Testing and Diagnostics
	**********************************************************/
	const bool speedModeTest = false; // oi->DL7->Pressed();
//	const bool distanceMode = oi->DL8->Pressed();
	const bool dmsMode = oi->DL11->Pressed();
	dmsProcessManager->SetRunning(dmsMode);

	const bool testFrontDrive = oi->DL9->Pressed();



	/**********************************************************
	 * Testing and Diagnostics
	**********************************************************/
	if (oi->DL6->Pressed()) {
		std::cout << "STOPPING AUTO\n";
		autoInitialized = false;
		Robot::visionSystem->GetLimelight()->SelectPipeline(0);
	}


	/**********************************************************
	 * Drive Control
	**********************************************************/
	double twistInput = oi->GetJoystickTwist(threshold);

	if (visionMode) {
		double currentYaw = RobotMap::gyro->GetYaw();
		double newYaw = calculateLockAngle(currentYaw);
		// std::cout <<" currentYaw = "<< currentYaw << " | newYaw = " << newYaw << "\n";
		driveBase->SetTargetAngle(newYaw);
		twistInput = driveBase->GetTwistControlOutput();
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
		// driveBase->SetConstantVelocity(twistInput, 0.60);
		// driveBase->Diagnostics();
		jackScrewTest.Run();
	} else if (dmsMode) {
		// DriveBase input handled via DMS->Run()
	} else if (runningLiftSequence) {
		liftController->Run();
	} else if (runningScrews) {
		// manual control
		jackScrews->ConfigureOpenLoop(-oi->GetJoystickY(threshold));
	} else if (testFrontDrive) {
		if (oi->DL9->RisingEdge()) {		// warning DL9 reused in liftControl->Run()
			driveBase->SetTargetAngle(180.0);
		}
		std::cout << "Running liftDrive\n";
		liftDrive.DriveFront(
			driveBase->GetCrabTwistOutput(),
			0.2,
			0,
			true);
	} else if (autoInitialized) {
		autoManager->Periodic(world);
	} else {
		if (!lockWheels) {
			const double yMove = -oi->GetJoystickY(threshold);
			double xMove = oi->GetJoystickX();
			bool useGyro = true;
			if (visionMode) { 
				// Ignore xMove if we are level2 position which blocks the camera or if we are moving backwards
				const bool notAtLevel2 = Elevator::ElevatorPosition::kLevel2 != elevator->GetElevatorPosition();
				const bool movingForward = yMove > 0.0;
				if (notAtLevel2 || movingForward) {
					xMove = visionSystem->GetLastVisionInfo()->xSpeed;
				}
				useGyro = false;
			} else if (oi->DR4->Pressed()) {
				// robot centric
				xMove = std::copysign(xMove*xMove, xMove);
				twistInput *= 0.5;
				useGyro = false;
			}
			driveBase->Crab(
				twistInput,
				yMove,
				xMove,
				useGyro);
		} else {
			driveBase->Crab(0, 0, 0, true);
		}
	}
	

	double now = frc::Timer::GetFPGATimestamp();
	double driveBaseTime = (now-start) * 1000;
	SmartDashboard::PutNumber("DriveBaseRun", driveBaseTime);
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
	intakeRotate->Init();
	crawler->Init();
	elevator->Init();
	// status & dms currently don't have init
	std::cout << "Robot::InitSubsystems <=\n";
}

void Robot::RunSubsystems() {
    double start = frc::Timer::GetFPGATimestamp();
	jackScrews->Run();
    dmsProcessManager->Run();
	intake->Run();
	intakeRotate->Run();
	crawler->Run();
	visionSystem->Run(); 
	elevator->Run();
	// liftController takes over driving so is in teleop loop
	double now = frc::Timer::GetFPGATimestamp();
	SmartDashboard::PutNumber("Subsystem Times", (now-start) * 1000);
}

void Robot::InstrumentSubsystems() {
	if (runInstrumentation) {
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

		// see DriveBase::Instrment for smartdashboard yaw 
		frc::SmartDashboard::PutNumber("Penguin Temp", RobotMap::gyro->GetPigeon()->GetTemp());
		frc::SmartDashboard::PutBoolean("AHRS Connected", ahrs->IsConnected());
		frc::SmartDashboard::PutNumber("AHRS Yaw", ahrs->GetYaw());
		frc::SmartDashboard::PutNumber("AHRS Temp", ahrs->GetTempC());

		driveBase->Instrument();
		jackScrews->Instrument();
		intake->Instrument();
		intakeRotate->Instrument();
		crawler->Instrument();
		elevator->Instrument();
		visionSystem->Instrument();
	}
}

void Robot::HandleGlobalInputs() {
	if (oi->DR9->RisingEdge()) {
		visionSystem->ToggleCameraMode();
	}
	if (oi->DR7->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::LimelightMain);
	} else if (oi->DR8->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::USBMain);
	} else if (oi->DR10->RisingEdge()) {
		visionSystem->GetLimelight()->SetStreamMode(Limelight::StreamMode::SideBySide);
	}

	// Only run instrumentation when button is pressed to avoid
	// network latency overhead
	runInstrumentation = oi->DL7->Pressed();
}


void Robot::TestInit() {}
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

