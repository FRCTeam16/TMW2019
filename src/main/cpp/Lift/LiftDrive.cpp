/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Lift/LiftDrive.h"
#include <frc/Timer.h>
#include "RobotMap.h"
#include <cmath>
#include "Subsystems/Drive/DriveBase.h"
#include "Robot.h"

LiftDrive::LiftDrive() {}

void LiftDrive::DriveFront() {
    const double startTime = frc::Timer::GetFPGATimestamp();
    const bool useGyro = true;

    Robot::driveBase->SetTargetAngle(-180.0);
    double twist = Robot::driveBase->GetCrabTwistOutput();

    double y = -0.2;
    double x = 0.0;
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
}

void LiftDrive::SetSteering(DriveInfo<double> setpoint) {
    // Get position offsets and inverse status from drivebase by reference

/**
    frontLeft->SetSteerEncoderSetpoint(setpoint.FL, positionOffsets.FL, inv.FL);
    frontRight->SetSteerEncoderSetpoint(setpoint.FR, positionOffsets.FR, inv.FR);
    
    // TODO Need mode switch maybe?
    rearLeft->SetSteerEncoderSetpoint(setpoint.RL, positionOffsets.RL, inv.RL);
    rearRight->SetSteerEncoderSetpoint(setpoint.RR, positionOffsets.RR, inv.RR);

    */
}