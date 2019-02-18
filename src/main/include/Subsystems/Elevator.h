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



class Elevator : SubsystemManager {
public:
	Elevator();
	virtual ~Elevator() = default;

	enum class ElevatorPosition {
		kFloor, /*kLevel1,*/ kLevel2, kLevel3
	};
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


private:
	std::shared_ptr<WPI_TalonSRX> elevatorMotor = RobotMap::elevatorMotor;
	ElevatorPosition elevatorPosition = ElevatorPosition::kFloor;
	double openLoopPercent = 0.0;
	int elevatorPositionThreshold;
	double setpoint = 0.0;
	RunMode runMode = RunMode::kManual;

	void SetElevatorSetpoint(int setpoint);	// deprecated?

	bool initializeFinished = false;
	int initializeScanCounts = 0;
	int kInitializeScanCountMax = 25;
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
