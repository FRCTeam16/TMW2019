/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/LiftDrive.h"
#include <frc/Timer.h>
#include "Robot.h"
#include "RobotMap.h"
#include <cmath>
#include "Subsystems/Drive/DriveBase.h"


LiftDrive::LiftDrive() {}

void LiftDrive::DriveTank(double leftInput, double rightInput) {
	std::cout << "LiftDrive::DriveTank(" << leftInput << ", " << rightInput << ")";
	DriveInfo<double> setpoint(0.0);
	SetSteering(setpoint);

	auto wheels = Robot::driveBase->GetWheels();

	int invFL = Robot::driveBase->inv.FL;
	int invFR = Robot::driveBase->inv.FR;
	std::cout << " | INV.FL = " << invFL << " - INV.FR = " << invFR << "\n";
	wheels.FL->UseOpenLoopDrive(leftInput * invFL);
	wheels.FR->UseOpenLoopDrive(rightInput * invFR);
}

void LiftDrive::DriveFront(double twist, double y, double x, bool useGyro) {
    const double startTime = frc::Timer::GetFPGATimestamp();
	double FWD = y;     // y input
	double STR = x;     // x input

	if (useGyro) {
		assert(RobotMap::gyro.get() != nullptr);
		const double robotangle = RobotMap::gyro->GetYaw() * M_PI / 180;
		FWD =  y * cos(robotangle) + x * sin(robotangle);
		STR = -y * sin(robotangle) + x * cos(robotangle);
	}

    const Wheelbase wheelbase = Robot::driveBase->wheelbase;
	const double radius = sqrt(pow(2*wheelbase.Y, 2) + pow(wheelbase.X,2));
	double AP = STR - twist * wheelbase.X / radius;
	double BP = STR + twist * wheelbase.X / radius;
	double CP = FWD - twist * 2 * wheelbase.Y / radius;
	double DP = FWD + twist * 2 * wheelbase.Y / radius;

	// ********************************************************************* //

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
	const double steerEnd = (startTime - frc::Timer::GetFPGATimestamp()) * -1000.0;

	// ********************************************************************* //

	const double driveStartTime = frc::Timer::GetFPGATimestamp();
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

	const double driveEnd = (driveStartTime - frc::Timer::GetFPGATimestamp()) * -1000.0;
	frc::SmartDashboard::PutNumber("Crab Drive (ms)", driveEnd);
	const double end = (startTime - frc::Timer::GetFPGATimestamp()) * -1000.0;
	frc::SmartDashboard::PutNumber("Crab Time (ms)", end);
}


void LiftDrive::SetSteering(DriveInfo<double> setpoint) {
	auto db = Robot::driveBase;
	db->frontLeft->SetSteerEncoderSetpoint(setpoint.FL, db->positionOffsets.FL, db->inv.FL);
	db->frontRight->SetSteerEncoderSetpoint(setpoint.FR, db->positionOffsets.FR, db->inv.FR);

	if (DriveMode::kAll == driveMode) {
		db->rearLeft->SetSteerEncoderSetpoint(setpoint.RL, db->positionOffsets.RL, db->inv.RL);
		db->rearRight->SetSteerEncoderSetpoint(setpoint.RR, db->positionOffsets.RR, db->inv.RR);
	}
}

void LiftDrive::SetDriveSpeed(DriveInfo<double> speed) {
	// Check one of our drives to see if the mode is open or closed loop
	// FIXME: RPM Scale factor needs to be in preferences
	auto db = Robot::driveBase;
	const bool isOpenLoop = db->frontLeft->IsOpenLoopDrive();	
	const float SCALE_FACTOR = (isOpenLoop) ? 1 : 6000;	// 13000 for Talon

	DriveInfo<double> speeds;
	if(driveFront) {
		speeds.FL = speed.FL * db->inv.FL * SCALE_FACTOR;
		speeds.FR = speed.FR * db->inv.FR * SCALE_FACTOR;
		speeds.RL = speed.RL * db->inv.RL * SCALE_FACTOR;
		speeds.RR = speed.RR * db->inv.RR * SCALE_FACTOR;
	} else {
		speeds.FL = speed.RR * db->inv.FL * SCALE_FACTOR;
		speeds.FR = speed.RL * db->inv.FR * SCALE_FACTOR;
		speeds.RL = speed.FR * db->inv.RL * SCALE_FACTOR;
		speeds.RR = speed.FL * db->inv.RR * SCALE_FACTOR;
	}
	if (isOpenLoop) {
		db->frontLeft->UseOpenLoopDrive(speeds.FL);
		db->frontRight->UseOpenLoopDrive(speeds.FR);
		if (DriveMode::kAll == driveMode) {
			db->rearLeft->UseOpenLoopDrive(speeds.RL);
			db->rearRight->UseOpenLoopDrive(speeds.RR);
		}
	} else {
		db->frontLeft->UseClosedLoopDrive(speeds.FL);
		db->frontRight->UseClosedLoopDrive(speeds.FR);
		if (DriveMode::kAll == driveMode) {
			db->rearLeft->UseClosedLoopDrive(speeds.RL);
			db->rearRight->UseClosedLoopDrive(speeds.RR);
		}
	}
}
