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
	virtual ~Elevator();

	enum class ElevatorPosition {
		kFloor, kLevel1, kLevel2, kLevel3
	};
	const int ELEVATOR_POSITION_COUNT = 4;

	enum RunMode {
		kManual, kMagic
	};


	void InitDefaultCommand() {}
	void Periodic() {}

	void Init() override;
	void Run() override;
	void Instrument() override;

	void SetOpenLoopPercent(double openLoopPercent);

	void SetInitialPosition();
	ElevatorPosition GetElevatorPosition();
	void SetElevatorPosition(ElevatorPosition position);
	void SetElevatorSetpoint(int setpoint);
	bool InPosition();
	void IncreaseElevatorPosition();
	void DecreaseElevatorPosition();
	void HoldPosition();

	void SetHomePosition();

	int GetElevatorEncoderPosition();
	std::tuple<double, double> GetElevatorMotorCurrents();


private:
	std::shared_ptr<WPI_TalonSRX> elevatorMotor = RobotMap::elevatorMotor;
	std::shared_ptr<WPI_TalonSRX> followerElevatorMotor;// = RobotMap::elevatorElevatorMotor2;
	const std::vector<std::shared_ptr<WPI_TalonSRX>> motors { elevatorMotor, followerElevatorMotor };
	ElevatorPosition elevatorPosition = ElevatorPosition::kFloor;
	double openLoopPercent = 0.0;
	int elevatorPositionThreshold;
	double setpoint = 0.0;
	RunMode runMode = RunMode::kManual;
	bool shifterState = false;       // false = reverse = low, true = forward = high

	void SetShift(bool state);
};

#endif /* SRC_SUBSYSTEMS_ELEVATOR_H_ */
