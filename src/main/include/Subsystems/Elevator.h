/*
 * Elevator.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_SUBSYSTEMS_ELEVATOR_H_
#define SRC_SUBSYSTEMS_ELEVATOR_H_
#include "SubsystemManager.h"
#include "frc/WPILib.h"
#include "ctre/Phoenix.h"
#include "RobotMap.h"
#include "ElevatorIntakeSetpointStrategy.h"



class Elevator : SubsystemManager {
public:
	Elevator();
	virtual ~Elevator() = default;

	enum class ElevatorPosition {
		kFloor, kLevel1, kLevel2, kLevel3, kSpecialCargo
	};
	// Elevator position count must not include special modes
	const int ELEVATOR_POSITION_COUNT = 4;

	enum RunMode {
		kManual, kMagic
	};

	void Init() override;
	void Run() override;
	void Instrument() override;
	void DisabledZeroOutput();

	void SetOpenLoopPercent(double openLoopPercent);

	void SetInitialPosition();
	ElevatorPosition GetElevatorPosition();
	void SetElevatorPosition(ElevatorPosition position);
	
	bool InPosition();
	void IncreaseElevatorPosition();
	void DecreaseElevatorPosition();
	void HoldPosition();

	void SetHomePosition();

	int GetElevatorEncoderPosition();
	double GetElevatorMotorCurrent();

	void ToggleCargoShotMode();

	void BumpUp();
	void BumpDown();


private:
	std::shared_ptr<WPI_TalonSRX> elevatorMotor = RobotMap::elevatorMotor;
	std::shared_ptr<WPI_TalonSRX> followerMotor = RobotMap::elevatorFollowerMotor;
	ElevatorIntakeSetpointStrategy elevatorSetpointStrategy;
	
	ElevatorPosition elevatorPosition = ElevatorPosition::kFloor;
	double openLoopPercent = 0.0;
	int elevatorPositionThreshold = 0;
	double setpoint = 0.0;
	RunMode runMode = RunMode::kManual;

	void SetElevatorSetpoint(int setpoint);	// deprecated?

	bool initializeFinished = false;
	int initializeScanCounts = 0;
	int kInitializeScanCountMax = 25;

	double setpointStartMoveTime = -1;
	double kFloorResetWaitPeriod = 1.0;		// time to wait before resetting to floor position

	bool cargoShotMode = false;
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
