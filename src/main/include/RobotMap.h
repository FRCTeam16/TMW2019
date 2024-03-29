/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include "Util/BSGyro.h"

using namespace std;

class RobotMap {
 public:
  RobotMap();
  static shared_ptr<rev::CANSparkMax> driveBaseFrontLeftDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseFrontLeftSteer;

  static shared_ptr<rev::CANSparkMax> driveBaseFrontRightDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseFrontRightSteer;

  static shared_ptr<rev::CANSparkMax> driveBaseRearLeftDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseRearLeftSteer;
  
  static shared_ptr<rev::CANSparkMax> driveBaseRearRightDrive;
  static shared_ptr<WPI_TalonSRX> driveBaseRearRightSteer;

	static std::shared_ptr<BSGyro> gyro;

  static std::shared_ptr<WPI_TalonSRX> elevatorMotor;
  static std::shared_ptr<WPI_TalonSRX> elevatorFollowerMotor;
  static std::shared_ptr<WPI_VictorSPX> crawlMotor;
  static std::shared_ptr<WPI_TalonSRX> rotateLeftMotor;
  static std::shared_ptr<WPI_VictorSPX> rotateRightMotor;
  static std::shared_ptr<WPI_TalonSRX> beaterTopMotor;
  static std::shared_ptr<WPI_VictorSPX> beaterBottomMotor;

  static std::shared_ptr<DoubleSolenoid> frontAxleSolenoid;
  static std::shared_ptr<DoubleSolenoid> rearAxleSolenoid;

  static std::shared_ptr<Solenoid> ejectorSolenoid;
  static std::shared_ptr<Solenoid> hatchCatchSolenoid;
  static std::shared_ptr<DoubleSolenoid> gripperSolenoid;

  static std::shared_ptr<Compressor> compressor;

  static std::shared_ptr<DoubleSolenoid> extendSuctionCup;
  static std::shared_ptr<DoubleSolenoid> openSuction;
};
