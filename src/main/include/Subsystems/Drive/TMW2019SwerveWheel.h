/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include <memory>
#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "Subsystems/Drive/SwerveWheel.h"

class TMW2019SwerveWheel : public SwerveWheel {
public:
  TMW2019SwerveWheel(std::shared_ptr<rev::CANSparkMax> driveMotor_, std::shared_ptr<WPI_TalonSRX> steerMotor_) 
    : driveMotor(driveMotor_), steerMotor(steerMotor_) {}

  void InitTeleop() override;
  void InitAuto() override;  
  void InitializeSteering() override;
  void InitializeDrivePID() override;

  bool IsOpenLoopDrive() override; 
  void UseOpenLoopDrive(double speed = 0.0) override;
  void UseClosedLoopDrive(double value = 0.0) override;
  double GetDriveEncoderPosition() override;
  double GetDriveVelocity() override;
  void ZeroDriveEncoder() override;
  double GetDriveOutputCurrent() override;
  bool HasCANError() override;
  std::shared_ptr<rev::CANSparkMax> GetDriveMotor();
  
  void UseOpenLoopSteer(double speed = 0.0) override;
  void UseClosedLoopSteer(double value = 0.0) override;
  double GetSteerEncoderPositionInDegrees() override; 
  void SetSteerEncoderSetpoint(double setpoint, double offset, int &inv) override;
  int GetSteerEncoderPosition() override;
  double GetSteerVelocity() override;
  double GetSteerOutputCurrent() override;
  std::shared_ptr<WPI_TalonSRX> GetSteerMotor();

  void SetDriveBrakeMode() override;
  void SetDriveCoastMode() override;

private:
  std::shared_ptr<rev::CANSparkMax> driveMotor;
  std::shared_ptr<WPI_TalonSRX> steerMotor;
  bool isOpenLoop = true;
  double kSteerP = 5.0; // was 10
};