/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Drive/TMW2019SwerveWheel.h"
#include "frc/Preferences.h"
#include <iostream>

void TMW2019SwerveWheel::InitializeSteering() {
    assert(steerMotor.get() != nullptr);
    steerMotor->SetNeutralMode(NeutralMode::Coast);
    steerMotor->Set(ControlMode::Position, 0);
    steerMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
    steerMotor->SetInverted(false);
    steerMotor->Config_kP(0, 1.0, 0);
    steerMotor->ConfigPeakOutputForward(0.75, 0);
    steerMotor->ConfigPeakOutputReverse(-0.75, 0);
}

void TMW2019SwerveWheel::InitializeDrivePID() {
    assert(driveMotor.get() != nullptr);
    frc::Preferences *prefs = frc::Preferences::GetInstance();

    if (!prefs->ContainsKey("DriveP")) {
        prefs->PutFloat("DriveP", 0.00);
    }
    if (!prefs->ContainsKey("DriveI")) {
        prefs->PutFloat("DriveI", 0.00);
    }
    if (!prefs->ContainsKey("DriveD")) {
        prefs->PutFloat("DriveD", 0.00);
    }
    if (!prefs->ContainsKey("DriveF")) {
        prefs->PutFloat("DriveF", 0.00);
    }
    if (!prefs->ContainsKey("DriveIZone")) {
        prefs->PutFloat("DriveIZone", 0.00);
    }
    const double driveP = prefs->GetFloat("DriveP");
	const double driveI = prefs->GetFloat("DriveI");
	const double driveD = prefs->GetFloat("DriveD");
	const double driveF = prefs->GetFloat("DriveF");
	const double driveIZone = prefs->GetFloat("DriveIZone");

    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    driveMotor->Set(0.0);
    isOpenLoop = true;


    rev::CANPIDController pid = driveMotor->GetPIDController();
    pid.SetP(driveP);    
    pid.SetI(driveI);
    pid.SetD(driveD);
    pid.SetFF(driveF);
    pid.SetIZone(driveIZone);
    pid.SetOutputRange(-1.0, 1.0);
}

bool TMW2019SwerveWheel::IsOpenLoopDrive() {
    return isOpenLoop;
}

double TMW2019SwerveWheel::GetDriveEncoderPosition() {
    return driveMotor->GetEncoder().GetPosition();
}

double TMW2019SwerveWheel::GetDriveVelocity() {
    return driveMotor->GetEncoder().GetVelocity();
}

void TMW2019SwerveWheel::ZeroDriveEncoder() {
    // talon motor->SetSelectedSensorPosition(0, 0, 0);
    //driveMotor->GetPIDController().  
    std::cout << "[ERROR] Request to Zero Drive Encoders, but SparkMax does not support that operation!\n";
}

double TMW2019SwerveWheel::GetDriveOutputCurrent() {
    return driveMotor->GetOutputCurrent();
}

void TMW2019SwerveWheel::InitTeleop() {
    steerMotor->Config_kP(0, 1.0, 0);
}

void TMW2019SwerveWheel::InitAuto() {
    steerMotor->Config_kP(0, 3.0, 0);
}

void TMW2019SwerveWheel::UseOpenLoopDrive(double speed) {
    // driveMotor->Set(ControlMode::PercentOutput, 0.0);
    driveMotor->Set(speed);
    isOpenLoop = true;
}

void TMW2019SwerveWheel::UseClosedLoopDrive(double value) {
    // driveMotor->Set(ControlMode::Velocity, 0);
    // std::cout << "TMW2019SwerveWheel::UseClosedLoopDrive(" << value << ")\n";

    frc::Preferences *prefs = frc::Preferences::GetInstance();
    const double driveP = prefs->GetFloat("DriveP");
	const double driveI = prefs->GetFloat("DriveI");
	const double driveD = prefs->GetFloat("DriveD");
	const double driveF = prefs->GetFloat("DriveF");
	const double driveIZone = prefs->GetFloat("DriveIZone");

    rev::CANPIDController pid = driveMotor->GetPIDController();
    pid.SetP(driveP);    
    pid.SetI(driveI);
    pid.SetD(driveD);
    pid.SetFF(driveF);
    pid.SetIZone(driveIZone);
    pid.SetOutputRange(-1.0, 1.0);

    pid.SetReference(value, rev::ControlType::kPosition);
    isOpenLoop = false;
}


double TMW2019SwerveWheel::GetSteerEncoderPositionInDegrees() {
    int currentPosition = steerMotor->GetSelectedSensorPosition(0);
	int currentPositionEncoderUnits = currentPosition % 4096;
	double positionInDegrees = (currentPositionEncoderUnits / 4096.0 * 360.0);

	if (positionInDegrees < -180.0) {
		positionInDegrees += 360.0;
	} else if (positionInDegrees > 180.0) {
		positionInDegrees -= 360.0;
	}
	return positionInDegrees;
}

void TMW2019SwerveWheel::SetSteerEncoderSetpoint(double setpoint, double offset, int &inv) {
    double currentPosition = steerMotor->GetSelectedSensorPosition(0) / 4096.0;
	double setpointRotations = (setpoint + offset) / 360.0;

	double wholeRotations = 0.0;
	double diff = modf(setpointRotations - currentPosition, &wholeRotations);
	if (fabs(diff) > 0.25) {
		diff -= copysign(0.5, diff);
		inv = -1;
	} else {
		inv = 1;
	}

	double finalSetpoint = currentPosition + diff;
//	std::cout << "current: " << currentPosition <<
//			" | setpoint: " << setpoint <<
//			" | diff: " << diff <<
//			" | final: " << finalSetpoint << std::endl;
	steerMotor->Set(ControlMode::Position, finalSetpoint * 4096);
}

int TMW2019SwerveWheel::GetSteerEncoderPosition() {
    return steerMotor->GetSelectedSensorPosition(0);
}

double TMW2019SwerveWheel::GetSteerVelocity() {
    return steerMotor->GetSelectedSensorVelocity(0);
}

double TMW2019SwerveWheel::GetSteerOutputCurrent() {
    return steerMotor->GetOutputCurrent();
}

void TMW2019SwerveWheel::UseOpenLoopSteer(double speed) {
    steerMotor->Set(ControlMode::PercentOutput, speed);
}

void TMW2019SwerveWheel::UseClosedLoopSteer(double value) {
    steerMotor->Set(ControlMode::Position, value);
}


