/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotMap.h"

  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseFrontLeftDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontLeftSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseFrontRightDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseFrontRightSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseRearLeftDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearLeftSteer;
  shared_ptr<rev::CANSparkMax> RobotMap::driveBaseRearRightDrive;
  shared_ptr<WPI_TalonSRX> RobotMap::driveBaseRearRightSteer;

  std::shared_ptr<BSGyro> RobotMap::gyro;

  std::shared_ptr<WPI_TalonSRX> RobotMap::elevatorMotor;
  std::shared_ptr<WPI_TalonSRX> RobotMap::elevatorFollowerMotor;
  std::shared_ptr<WPI_VictorSPX> RobotMap::crawlMotor;
  std::shared_ptr<WPI_TalonSRX> RobotMap::rotateLeftMotor;
  std::shared_ptr<WPI_VictorSPX> RobotMap::rotateRightMotor;
  std::shared_ptr<WPI_TalonSRX> RobotMap::beaterTopMotor;
  std::shared_ptr<WPI_VictorSPX> RobotMap::beaterBottomMotor;

  std::shared_ptr<DoubleSolenoid> RobotMap::frontAxleSolenoid;
  std::shared_ptr<DoubleSolenoid> RobotMap::rearAxleSolenoid;
  
  std::shared_ptr<Solenoid> RobotMap::ejectorSolenoid;
  std::shared_ptr<Solenoid> RobotMap::hatchCatchSolenoid;
  std::shared_ptr<Solenoid> RobotMap::gripperSolenoid;
  
  std::shared_ptr<Compressor> RobotMap::compressor;



RobotMap::RobotMap() {
  driveBaseFrontLeftDrive.reset(new rev::CANSparkMax{1, rev::CANSparkMaxLowLevel::MotorType::kBrushless});
  driveBaseFrontRightDrive.reset(new rev::CANSparkMax{3, rev::CANSparkMaxLowLevel::MotorType::kBrushless});
  driveBaseRearLeftDrive.reset(new rev::CANSparkMax{5, rev::CANSparkMaxLowLevel::MotorType::kBrushless});
  driveBaseRearRightDrive.reset(new rev::CANSparkMax{7, rev::CANSparkMaxLowLevel::MotorType::kBrushless});

  driveBaseFrontLeftSteer.reset(new WPI_TalonSRX{2});
  driveBaseFrontRightSteer.reset(new WPI_TalonSRX{4});
  driveBaseRearLeftSteer.reset(new WPI_TalonSRX{6});
  driveBaseRearRightSteer.reset(new WPI_TalonSRX{8});

  elevatorMotor.reset(new WPI_TalonSRX{9});
  elevatorFollowerMotor.reset(new WPI_TalonSRX{15});
  crawlMotor.reset(new WPI_VictorSPX{10});
  rotateLeftMotor.reset(new WPI_TalonSRX{11});
  rotateRightMotor.reset(new WPI_VictorSPX{12});
  beaterTopMotor.reset(new WPI_TalonSRX{13});
  beaterBottomMotor.reset(new WPI_VictorSPX{14});

  gyro.reset(new BSGyro(elevatorFollowerMotor.get()));  

  frontAxleSolenoid.reset(new DoubleSolenoid{0, 1});
  rearAxleSolenoid.reset(new DoubleSolenoid{2, 3});
  ejectorSolenoid.reset(new Solenoid{4});
  gripperSolenoid.reset(new Solenoid{5});
  hatchCatchSolenoid.reset(new Solenoid{6});


  compressor.reset(new Compressor{0});
  compressor->SetClosedLoopControl(true);
}
