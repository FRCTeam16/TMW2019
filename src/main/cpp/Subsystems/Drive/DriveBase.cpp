// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


#include <iostream>
#include <vector>

#include "ctre/Phoenix.h"
#include "RobotMap.h"
#include "Subsystems/Drive/DriveBase.h"
#include "Subsystems/Drive/CrabSpeed.h"
#include "Subsystems/Drive/DriveEncoderPIDSource.h"
#include "Util/PrefUtil.h"
#include "Subsystems/Drive/TMW2019SwerveWheel.h"

# define M_PI		3.14159265358979323846	/* pi */

//#define DEBUG 1

#ifdef DEBUG
#define D(x) x
#define DM(x) (std::cout << x << std::endl)
#else
#define D(x)
#define DM(x)
#endif


using namespace frc;

DriveBase::DriveBase() : Subsystem("DriveBase") {
	std::cout << "DriveBase::DriveBase =>\n";

	frontLeft.reset(new TMW2019SwerveWheel{RobotMap::driveBaseFrontLeftDrive, RobotMap::driveBaseFrontLeftSteer});
	frontRight.reset(new TMW2019SwerveWheel{RobotMap::driveBaseFrontRightDrive, RobotMap::driveBaseFrontRightSteer});
	rearLeft.reset(new TMW2019SwerveWheel{RobotMap::driveBaseRearLeftDrive, RobotMap::driveBaseRearLeftSteer});
	rearRight.reset(new TMW2019SwerveWheel{RobotMap::driveBaseRearRightDrive, RobotMap::driveBaseRearRightSteer});
	wheels.push_back(frontLeft);
	wheels.push_back(frontRight);
	wheels.push_back(rearLeft);
	wheels.push_back(rearRight);
	std::cout << "  wheels initialized\n";

    crabSpeedTwist.reset(new CrabSpeed());

	InitializeOffsets();
	std::cout << "  offsets initialized\n";

    wheelbase.W = 23.5/2;
    wheelbase.X = 27.5;
    wheelbase.Y = 23.5/2;

    // Initialize wheels
	for (auto const& wheel : wheels) {
		wheel->InitializeSteering();
	}
	std::cout << "  steering initialized\n";
    InitializePIDs();
	std::cout << "DriveBase::DriveBase <=";
}

void DriveBase::InitializePIDs() {
	 // Initialize Drive Talons + PID feedback
	Preferences *prefs = Preferences::GetInstance();
	const double izone = PrefUtil::getSet("DriveControlTwistIZone", 0.0);

	if (driveControlTwist == nullptr) {
		driveControlTwist.reset(
					new PIDController(
							prefs->GetFloat("TwistP", 0.02),
							prefs->GetFloat("TwistI", 0.0),
							prefs->GetFloat("TwistD", 0.12),
							0.0, RobotMap::gyro.get(), crabSpeedTwist.get(), 0.02 ));
	} else {
		driveControlTwist->SetPID(
				prefs->GetFloat("TwistP", 0.02),
				prefs->GetFloat("TwistI", 0.0),
				prefs->GetFloat("TwistD", 0.12));
		// was setting izone in custom PID
	}
	driveControlTwist->SetContinuous(true);
	driveControlTwist->SetAbsoluteTolerance(2.0);
	driveControlTwist->Enable();
	driveControlTwist->SetOutputRange(-0.5, 0.5);
	driveControlTwist->SetInputRange(-180, 180);

	// Drive Motor PID	
	for (auto const& wheel : wheels) {
		wheel->InitializeDrivePID();
	}

	// Drive PID Control
	const double driveControlP = prefs->GetFloat("DriveControlP");
	const double driveControlI = prefs->GetFloat("DriveControlI");
	const double driveControlD = prefs->GetFloat("DriveControlD");
	const double driveControlF = prefs->GetFloat("DriveControlF");
	const double driveControlIZone = prefs->GetFloat("DriveControlIZone");
	if (driveControlEncoderSource == nullptr) {
		DriveInfo<std::shared_ptr<SwerveWheel>> motors;
		motors.FL = frontLeft;
		motors.FR = frontRight;
		motors.RL = rearLeft;
		motors.RR = rearRight;
		driveControlEncoderSource.reset(new FrontTwoAveragingDriveEncoderPIDSource(motors));
	}

	if (driveControlDistanceSpeed == nullptr) {
		driveControlDistanceSpeed.reset(new CrabSpeed());
	}

	if (driveControlSpeedController == nullptr) {
		driveControlSpeedController.reset(
				new PIDController(driveControlP, driveControlI, driveControlD, driveControlF,
						driveControlEncoderSource.get(),
						driveControlDistanceSpeed.get(),
						0.05));
//		driveControlSpeedController->SetIzone(driveControlIZone);
		driveControlSpeedController->Enable();
	} else {
		driveControlSpeedController->SetPID(driveControlP, driveControlI, driveControlD, driveControlF);
//		driveControlSpeedController->SetIzone(driveControlIZone);
	}
}

void DriveBase::InitDefaultCommand() {
}


// Put methods for controlling this subsystem
// here. Call these from Commands.

void DriveBase::InitializeOffsets() {
	positionOffsets.FL = PrefUtil::getSet("FLOff", 0.0);
	positionOffsets.FR = PrefUtil::getSet("FROff", 0.0);
	positionOffsets.RL = PrefUtil::getSet("RLOff", 0.0);
	positionOffsets.RR = PrefUtil::getSet("RROff", 0.0);
}

void DriveBase::Lock() {
	DriveInfo<double> steering;
	steering.FL = 0.375;
	steering.FR = 0.75;
	steering.RL = 0.75;
	steering.RR = 4.5;	// FIXME: Units need updating
	SetSteering(steering);

	DriveInfo<double> lockSpeed;
	SetDriveSpeed(lockSpeed);
}

DriveInfo<double> DriveBase::CalculatePositionOffsets() {
	positionOffsets.FL = frontLeft->GetSteerEncoderPositionInDegrees();
	positionOffsets.FR = frontRight->GetSteerEncoderPositionInDegrees();
	positionOffsets.RL = rearLeft->GetSteerEncoderPositionInDegrees();
	positionOffsets.RR = rearRight->GetSteerEncoderPositionInDegrees();
	return positionOffsets;
}


void DriveBase::ZeroTurnInfo() {
	turns.FL = 0;
	turns.FR = 0;
	turns.RL = 0;
	turns.RR = 0;
}

void DriveBase::ZeroDriveEncoders() {
	for (auto const& wheel : wheels) {
		wheel->ZeroDriveEncoder();
	}
}

void DriveBase::InitTeleop() {
	InitializePIDs();
	this->UseOpenLoopDrive();
	for (auto const& wheel : wheels) {
		wheel->InitTeleop();
	}
}

void DriveBase::InitAuto() {
	InitializePIDs();
	UseClosedLoopDrive();
	for (auto const& wheel : wheels) {
		wheel->InitAuto();
	}
}

void DriveBase::UseOpenLoopDrive() {
	std::cout << "*** UseOpenLoopDrive ***\n";
	for (auto const& wheel : wheels) {
		wheel->UseOpenLoopDrive();
	}
}


void DriveBase::UseClosedLoopDrive() {
	std::cout << "*** UseClosedLoopDrive ***\n";
	for (auto const& wheel : wheels) {
		wheel->UseClosedLoopDrive();
	}
}

void DriveBase::Crab(double twist, double y, double x, bool useGyro) {
	lastSpeedX = x;
	lastSpeedY = y;
	float FWD = y;
	float STR = x;

	if (useGyro) {
		assert(RobotMap::gyro.get() != nullptr);
		const double robotangle = RobotMap::gyro->GetYaw() * M_PI / 180;
		FWD =  y * cos(robotangle) + x * sin(robotangle);
		STR = -y * sin(robotangle) + x * cos(robotangle);
	}

	const double radius = sqrt(pow(2*wheelbase.Y, 2) + pow(wheelbase.X,2));
	double AP = STR - twist * wheelbase.X / radius;
	double BP = STR + twist * wheelbase.X / radius;
	double CP = FWD - twist * 2 * wheelbase.Y / radius;
	double DP = FWD + twist * 2 * wheelbase.Y / radius;


	DriveInfo<double> setpoint(0.0);

	if (DP != 0 || BP != 0) {
		setpoint.FL = atan2(BP,DP) * (180/M_PI);
	}
	if (BP != 0 || CP != 0) {
		setpoint.FR = atan2(BP, CP) * (180/M_PI);
	}
	if (AP != 0 || DP != 0) {
		setpoint.RL = atan2(AP, DP) * (180/M_PI);
	}
	if (AP != 0 || CP != 0) {
		setpoint.RR = atan2(AP, CP) * (180/M_PI);
	}

	SetSteering(setpoint);


	DriveInfo<double> speed(0.0);
	speed.FL = sqrt(pow(BP, 2) + pow(DP, 2));
	speed.FR = sqrt(pow(BP, 2) + pow(CP, 2));
	speed.RL = sqrt(pow(AP, 2) + pow(DP, 2));
	speed.RR = sqrt(pow(AP, 2) + pow(CP, 2));

	double speedarray[] = {fabs(speed.FL), fabs(speed.FR), fabs(speed.RL), fabs(speed.RR)};
	const double maxspeed = *std::max_element(speedarray, speedarray+4);

	DriveInfo<double> ratio;
	if (maxspeed > 1 || maxspeed < -1) {
		ratio.FL = speed.FL / maxspeed;
		ratio.FR = speed.FR / maxspeed;
		ratio.RL = speed.RL / maxspeed;
		ratio.RR = speed.RR / maxspeed;
	} else {
		ratio.FL = speed.FL;
		ratio.FR = speed.FR;
		ratio.RL = speed.RL;
		ratio.RR = speed.RR;
	}
	ratio.FR = -ratio.FR;
	ratio.RR = -ratio.RR;
	SetDriveSpeed(ratio);
}

void DriveBase::SetSteering(DriveInfo<double> setpoint) {
	if (driveFront) {
		DM("FL");
		frontLeft->SetSteerEncoderSetpoint(setpoint.FL, positionOffsets.FL, inv.FL);
		DM("FR");
		frontRight->SetSteerEncoderSetpoint(setpoint.FR, positionOffsets.FR, inv.FR);
		DM("RL");
		rearLeft->SetSteerEncoderSetpoint(setpoint.RL, positionOffsets.RL, inv.RL);
		DM("RR");
		rearRight->SetSteerEncoderSetpoint(setpoint.RR, positionOffsets.RR, inv.RR);
	} else {
		frontLeft->SetSteerEncoderSetpoint(setpoint.RR, positionOffsets.FL, inv.FL);
		frontRight->SetSteerEncoderSetpoint(setpoint.RL, positionOffsets.FR, inv.FR);
		rearLeft->SetSteerEncoderSetpoint(setpoint.FR, positionOffsets.RL, inv.RL);
		rearRight->SetSteerEncoderSetpoint(setpoint.FL, positionOffsets.RR, inv.RR);
	}
}


void DriveBase::SetSteerSetpoint(double setpoint,
		std::shared_ptr<AnalogInput> actual, double offset,
		std::shared_ptr<PIDController> PIDCon, std::shared_ptr<WPI_TalonSRX> steer,
		int turns, int &inv) {
	const double volt = actual->GetVoltage();

	if (turns >= MaxTurns) {
		PIDCon->Disable();
		steer->Set(-1);
	} else if (turns <= -MaxTurns) {
		PIDCon->Disable();
		steer->Set(1);
	} else if (fabs(CorrectSteerSetpoint(setpoint + offset) - volt) < 1.25
			|| fabs(CorrectSteerSetpoint(setpoint + offset) - volt) > 3.75) {
		PIDCon->Enable();
		if ((turns + 1 == MaxTurns
				&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint) > 2.5)
				|| (turns - 1 == -MaxTurns
						&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint) < -2.5)) {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset - 2.5));
			inv = -1;
			D(std::cout << "SetSteer Inv #1 = -1" << std::endl;)
		} else {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset));
			inv = 1;
			D(std::cout << "SetSteer Inv #2 = 1" << std::endl;)
		}
	} else {
		PIDCon->Enable();
		if ((turns + 1 == MaxTurns
				&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint - 2.5) > 2.5)
				|| (turns - 1 == -MaxTurns
						&& CorrectSteerSetpoint(volt - offset) - CorrectSteerSetpoint(setpoint - 2.5) < -2.5)) {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset));
			inv = 1;
			D(std::cout << "SetSteer Inv #3 = 1" << std::endl;)
		} else {
			PIDCon->SetSetpoint(CorrectSteerSetpoint(setpoint + offset - 2.5));
			D(std::cout << "SetSteer Inv #4 = -1" << std::endl;)
			inv = -1;
		}
	}
}

double DriveBase::CorrectSteerSetpoint(double setpoint) {
	//Used to correct steering setpoints to within the 0 to 5 V scale
	if (setpoint < 0) {
		return setpoint + 5;
	} else if (setpoint > 5) {
		return setpoint - 5;
	} else if (setpoint == 5) {
		return 0;
	} else {
		return setpoint;
	}
}


void DriveBase::SetDriveSpeed(DriveInfo<double> speed) {
	const float driveOutLimit = 50;
	const int hotCountLimit = 100;
	const int coolCountLimit = 1000;

	frontLeft->GetDriveOutputCurrent() > driveOutLimit ? hotCount.FL++ : hotCount.FL =0;
	frontRight->GetDriveOutputCurrent() > driveOutLimit ? hotCount.FR++ : hotCount.FR =0;
	rearLeft->GetDriveOutputCurrent() > driveOutLimit ? hotCount.RL++ : hotCount.RL =0;
	rearRight->GetDriveOutputCurrent() > driveOutLimit ? hotCount.RR++ : hotCount.RR =0;

	if (hotCount.FL == 0 && hotCount.FR == 0 && hotCount.RL == 0 && hotCount.RR == 0 ) {
		coolCount++;
	} else {
		coolCount = 0;
	}

	if (hotCount.FL > hotCountLimit && hotCount.FR > hotCountLimit &&
			hotCount.RL > hotCountLimit  && hotCount.RR > hotCountLimit) {
		driveLimit = 0.5;
	}

	if (coolCount > coolCountLimit) {
		driveLimit = 1.0;
	}

	// Check one of our drives to see if the mode is open or closed loop
	// FIXME: RPM Scale factor needs to be in preferences
	const bool isOpenLoop = frontLeft->IsOpenLoopDrive();	
	const float SCALE_FACTOR = (isOpenLoop) ? 1 : 6000;	// 13000 for Talon

	DriveInfo<double> speeds;
	
	if(driveFront) {
		speeds.FL = speed.FL * inv.FL * SCALE_FACTOR;
		speeds.FR = speed.FR * inv.FR * SCALE_FACTOR;
		speeds.RL = speed.RL * inv.RL * SCALE_FACTOR;
		speeds.RR = speed.RR * inv.RR * SCALE_FACTOR;
	} else {
		speeds.FL = speed.RR * inv.FL * SCALE_FACTOR;
		speeds.FR = speed.RL * inv.FR * SCALE_FACTOR;
		speeds.RL = speed.FR * inv.RL * SCALE_FACTOR;
		speeds.RR = speed.FL * inv.RR * SCALE_FACTOR;
	}
	if (isOpenLoop) {
		frontLeft->UseOpenLoopDrive(speeds.FL);
		frontRight->UseOpenLoopDrive(speeds.FR);
		rearLeft->UseOpenLoopDrive(speeds.RL);
		rearRight->UseOpenLoopDrive(speeds.RR);
	} else {
		frontLeft->UseClosedLoopDrive(speeds.FL);
		frontRight->UseClosedLoopDrive(speeds.FR);
		rearLeft->UseClosedLoopDrive(speeds.RL);
		rearRight->UseClosedLoopDrive(speeds.RR);
	}
}

void DriveBase::SetConstantVelocity(double twistInput, double velocity) {
	Crab(twistInput, velocity, 0, true);
}

void DriveBase::SetTargetAngle(double angle) {
	driveControlTwist->SetSetpoint(angle);
}

double DriveBase::GetTwistControlOutput() {
//	return driveControlTwist->Get();
	return crabSpeedTwist->Get();
}

double DriveBase::GetTwistControlError() {
	return driveControlTwist->GetError();
}

void DriveBase::SetTargetDriveDistance(double distance, double maxSpeed) {
	driveControlSpeedController->SetSetpoint(distance);
	driveControlSpeedController->SetOutputRange(-maxSpeed, maxSpeed);
	driveControlEncoderSource->SetInitialEncoderValue();
}

double DriveBase::GetDriveControlSetpoint() {
	return driveControlSpeedController->GetSetpoint();
}

double DriveBase::GetDriveControlEncoderPosition() {
	return driveControlEncoderSource->PIDGet();
}

double DriveBase::GetDriveControlOutput() {
	return driveControlDistanceSpeed->Get();
}

double DriveBase::GetDriveControlError() {
	return driveControlSpeedController->GetError();
}

double DriveBase::GetDriveControlP() {
	return driveControlSpeedController->GetP();
}

Wheelbase DriveBase::GetWheelbase() {
	return wheelbase;
}

double DriveBase::GetCrabTwistOutput() {
	return crabSpeedTwist->Get();
}

void DriveBase::Instrument() {
/*	SmartDashboard::PutNumber("FL V", frontLeftDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("FR V", frontRightDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("RL V", rearLeftDrive->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("RR V", rearRightDrive->GetSelectedSensorVelocity(0));

	SmartDashboard::PutNumber("FL SErr", frontLeftSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("FR SErr", frontRightSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("RL SErr", rearLeftSteer->GetClosedLoopError(0));
	SmartDashboard::PutNumber("RR SErr", rearRightSteer->GetClosedLoopError(0));

	SmartDashboard::PutNumber("FL Pos", frontLeftDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("FR Pos", frontRightDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("RL Pos", rearLeftDrive->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("RR Pos", rearRightDrive->GetSelectedSensorPosition(0));
	*/
	SmartDashboard::PutNumber("RawYaw",RobotMap::gyro->ReadYaw());
	SmartDashboard::PutNumber("Yaw",RobotMap::gyro->GetYaw());
}

void DriveBase::Diagnostics() {
	frc::SmartDashboard::PutNumber("FL V", frontLeft->GetDriveVelocity());
	frc::SmartDashboard::PutNumber("FR V", frontRight->GetDriveVelocity());
	frc::SmartDashboard::PutNumber("RL V", rearLeft->GetDriveVelocity());
	frc::SmartDashboard::PutNumber("RR V", rearRight->GetDriveVelocity());

	frc::SmartDashboard::PutNumber("FL A", frontLeft->GetDriveOutputCurrent());
	frc::SmartDashboard::PutNumber("FR A", frontRight->GetDriveOutputCurrent());
	frc::SmartDashboard::PutNumber("RL A", rearLeft->GetDriveOutputCurrent());
	frc::SmartDashboard::PutNumber("RR A", rearRight->GetDriveOutputCurrent());
}

DriveInfo<int> DriveBase::GetDriveEncoderPositions() {
	DriveInfo<int> info;
	info.FL = frontLeft->GetDriveEncoderPosition();
	info.FR = frontRight->GetDriveEncoderPosition();
	info.RR = rearRight->GetDriveEncoderPosition();
	info.RL = rearLeft->GetDriveEncoderPosition();
	return info;
}

DriveInfo<double> DriveBase::GetDriveCurrent() {
	DriveInfo<double> info;
	info.FL = frontLeft->GetDriveOutputCurrent();
	info.FR = frontRight->GetDriveOutputCurrent();
	info.RR = rearRight->GetDriveOutputCurrent();
	info.RL = rearLeft->GetDriveOutputCurrent();
	return info;
}
 DriveInfo <int> DriveBase:: GetSteerEncoderPositions() {
 	 DriveInfo<int> info;
 	 info.FL = frontLeft->GetSteerEncoderPosition();
 	 info.FR = frontRight->GetSteerEncoderPosition();
 	 info.RR = rearRight->GetSteerEncoderPosition();
 	 info.RL = rearLeft->GetSteerEncoderPosition();
 	 return info;
 }

 DriveInfo <double> DriveBase::GetSteerCurrent() {
 	 DriveInfo<double> info;
 	 info.FL = frontLeft->GetSteerOutputCurrent();
 	 info.FR = frontRight->GetSteerOutputCurrent();
 	 info.RR = rearRight->GetSteerOutputCurrent();
 	 info.RL = rearLeft->GetSteerOutputCurrent();
 	 return info;
}

void DriveBase::DMSDrive(double speed) {
	for (auto const& wheel: wheels) {
		wheel->UseOpenLoopDrive(speed);
	}
}

void DriveBase::DMSSteer(double speed) {
	for(auto const& wheel : wheels) {
		wheel->UseOpenLoopSteer(speed);
	}
}

DriveInfo<int> DriveBase::GetDMSDriveVelocity() {
	DriveInfo<int> info;
	info.FL = frontLeft->GetDriveVelocity();
	info.FR = frontRight->GetDriveVelocity();
	info.RR = rearRight->GetDriveVelocity();
	info.RL = rearLeft->GetDriveVelocity();
	return info;
}

DriveInfo<int> DriveBase::GetDMSSteerVelocity() {
	DriveInfo<int> info;
	info.FL = frontLeft->GetSteerVelocity();
	info.FR = frontRight->GetSteerVelocity();
	info.RR = rearRight->GetSteerVelocity();
	info.RL = rearLeft->GetSteerVelocity();
	return info;
}


