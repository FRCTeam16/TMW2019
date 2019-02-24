/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Drive/TMW2019SwerveWheel.h"
#include "frc/Preferences.h"
#include <iostream>
#include "rev/CANError.h"
#include "Util/PrefUtil.h"
#include <string>

void TMW2019SwerveWheel::InitializeSteering() {
    assert(steerMotor.get() != nullptr);
    steerMotor->SetNeutralMode(NeutralMode::Coast);
    steerMotor->Set(ControlMode::Position, 0);
    steerMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 0);
    steerMotor->SetInverted(false);
    steerMotor->Config_kP(0, kSteerP, 0);
    steerMotor->ConfigPeakOutputForward(0.75, 0);
    steerMotor->ConfigPeakOutputReverse(-0.75, 0);
    steerMotor->ConfigPeakCurrentLimit(0);
    steerMotor->ConfigContinuousCurrentLimit(PrefUtil::getSet("Steer.ContinuousCurrentLimit", 20.0));
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
    steerMotor->Config_kP(0, kSteerP, 0);
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

std::shared_ptr<rev::CANSparkMax> TMW2019SwerveWheel::GetDriveMotor() {
    return driveMotor;
}

bool TMW2019SwerveWheel::HasCANError() {
    // TOOD: Is this the best way to get CANError info?  Seems like others
    // require sending configuration information
    rev::CANError error = driveMotor->ClearFaults();
    return error != rev::CANError::kOK;
}


// ****************************************************************************//

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
    double baseCurrentPosition = steerMotor->GetSelectedSensorPosition(0);
    double currentPosition = baseCurrentPosition / 4096.0;
	double setpointRotations = (setpoint + offset) / 360.0;

    // double spRotations = 0.0;
    // double spDiff = modf(setpointRotations, &spRotations);
	
    // double cpRotations = 0.0;
    // double cpDiff = modf(currentPosition, &cpRotations);
    // double initCpDiff = cpDiff;

    // double spRotations = 0.0;
    // double spDiff = modf(setpointRotations, &spRotations);

    // // Unify windings/signs
    // double rawDiff = setpointRotations - cpDiff;
    
	// double diff = spDiff - cpDiff;
    double wholeRotations = 0.0;
    // double diff = modf(setpointRotations - cpDiff, &wholeRotations);
    double diff = modf(setpointRotations - currentPosition, &wholeRotations);

    // Normalize difference into the nearest half of circle (positive or negative)
    if (fabs(diff) > 0.5) {
        if (diff < 0 ) {
            diff = 1.0 + diff;
        } else {
            diff = -1.0 + diff;
        }
    }

    // Invert wheel if difference > 90 degrees
	if (fabs(diff) > 0.25) {
		diff -= copysign(0.5, diff);
		inv = -1;
	} else {
		inv = 1;
	}
    
	double finalSetpoint = currentPosition + diff;
    // if (name == "FR") {
    //     std::cout << name << 
    //             ": base: " << baseCurrentPosition <<
    //             " | setpoint: " << setpoint <<
    //             " | curPos: " << currentPosition <<
    //             " | spRot: " << setpointRotations <<
    //             " | cpDiff: " << cpDiff <<
    //             " | diff: " << diff <<
    //             " | final: " << finalSetpoint << std::endl;
    // }
	steerMotor->Set(ControlMode::Position, finalSetpoint * 4096.0);

    // steerLog.Log(
    //     frc::Timer::GetFPGATimestamp(),
    //     baseCurrentPosition,
    //     setpoint,
    //     currentPosition,
    //     setpointRotations,
    //     initCpDiff,
    //     cpDiff,
    //     rawDiff,
    //     diff,
    //     wholeRotations,
    //     finalSetpoint);
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

std::shared_ptr<WPI_TalonSRX> TMW2019SwerveWheel::GetSteerMotor() {
    return steerMotor;
}


void TMW2019SwerveWheel::SetDriveBrakeMode() {
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void TMW2019SwerveWheel::SetDriveCoastMode() {
    driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
}
