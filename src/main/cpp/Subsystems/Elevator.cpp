#include <iostream>
#include "Subsystems/Elevator.h"
#include "Util/PrefUtil.h"
#include <frc/Timer.h>
#include "Robot.h"

Elevator::Elevator() {
	std::cout << "Elevator starting\n";

	elevatorMotor->SetNeutralMode(NeutralMode::Brake);
	elevatorMotor->ConfigPeakOutputForward(1);
	elevatorMotor->ConfigPeakOutputReverse(-1);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute);
	elevatorMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
	elevatorMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
	elevatorMotor->ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 1, 0, 0, 0);
	// elevatorMotor->SetSensorPhase(true);
	std::cout << "Elevator() complete\n";
}

void Elevator::Init() {
	elevatorPositionThreshold = PrefUtil::getSetInt("Elevator.Pos.Threshold", 10);
	SetInitialPosition();
}


void Elevator::SetInitialPosition() {
	runMode = RunMode::kManual;
	openLoopPercent = 0.0;
	elevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
}


void Elevator::Run() {
	switch (runMode) {
		case kManual:
			elevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
			break;
		case kMagic:
			double target = moveRequest != nullptr ? moveRequest->CalculateSetpoint() : setpoint;
			if (moveRequest->finished) { moveRequest.reset(); }

			double P = PrefUtil::getSet("Elevator.P", 0.01);
			double F = PrefUtil::getSet("Elevator.F", 0.18);
			int V = PrefUtil::getSetInt("Elevator.V", 5592);
			int A = PrefUtil::getSetInt("Elevator.A", 5592);

			elevatorMotor->ConfigMotionCruiseVelocity(V, 0);
			elevatorMotor->ConfigMotionAcceleration(A, 0);
			elevatorMotor->Config_kP(0, P, 0);
			elevatorMotor->Config_kI(0, 0, 0);
			elevatorMotor->Config_kD(0, 0, 0);
			elevatorMotor->Config_kF(0, F, 0);

			elevatorMotor->Set(ControlMode::MotionMagic, target);
			break;
	}
}

void Elevator::SetOpenLoopPercent(double _openLoopPercent) {
	runMode = RunMode::kManual;
	openLoopPercent = _openLoopPercent;
}

Elevator::ElevatorPosition Elevator::GetElevatorPosition() {
	return elevatorPosition;
}

void Elevator::SetElevatorPosition(ElevatorPosition _elevatorPosition) {
	runMode = RunMode::kMagic;
	elevatorPosition = _elevatorPosition;

	double lastSetpoint = setpoint;
	switch(elevatorPosition) {
		case ElevatorPosition::kFloor:
			setpoint = PrefUtil::getSet("Elevator.pos.Floor", 4000);
			break;
		case ElevatorPosition::kLevel1:
			setpoint = PrefUtil::getSet("Elevator.pos.Level1", 4200);
			break;
		case ElevatorPosition::kLevel2:
			setpoint = PrefUtil::getSet("Elevator.pos.Level2", 5000);
			break;
		case ElevatorPosition::kLevel3:
			setpoint = PrefUtil::getSet("Elevator.pos.Level3", 6000);
			break;
	}
	moveRequest.reset(new ElevatorMoveRequest(Robot::intakeRotate, lastSetpoint, setpoint));

	std::cout << "Elevator::SetElevatorPosition(" << static_cast<int>(elevatorPosition) << " | " << setpoint << "\n";
}

// Hidden/deprecated
void Elevator::SetElevatorSetpoint(int _setpoint) {
	setpoint = _setpoint;
}


bool Elevator::InPosition() {
    double error = setpoint - GetElevatorEncoderPosition();
    bool inPosition = (abs(error) < elevatorPositionThreshold);
//	std::cout << "Elevator in position: " << inPosition << "\n";
	return inPosition;
}


void Elevator::IncreaseElevatorPosition() {
	int nextOrdinal = static_cast<int>(elevatorPosition)  + 1;
	if (nextOrdinal < ELEVATOR_POSITION_COUNT) {
		ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
		SetElevatorPosition(nextPosition);
	}
}


void Elevator::DecreaseElevatorPosition() {
	int nextOrdinal = static_cast<int>(elevatorPosition) - 1;
	if (nextOrdinal >= 0 ) {
		ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
		SetElevatorPosition(nextPosition);
	}
}


void Elevator::HoldPosition() {
	openLoopPercent = 0.0;
	if (RunMode::kManual == runMode) {
		setpoint = GetElevatorEncoderPosition();
		runMode = RunMode::kMagic;
	}
}


void Elevator::SetHomePosition() {
	elevatorMotor->SetSelectedSensorPosition(0);
	setpoint = 0;
}

void Elevator::Instrument() {
	SmartDashboard::PutNumber("Elevator Position", GetElevatorEncoderPosition());
	SmartDashboard::PutNumber("Elevator Current (Main)", elevatorMotor->GetOutputCurrent());
	SmartDashboard::PutNumber("Elevator Output (Main)", elevatorMotor->Get());
	SmartDashboard::PutBoolean("Elevator Limit Switch FWD", elevatorMotor->GetSensorCollection().IsFwdLimitSwitchClosed());
	SmartDashboard::PutBoolean("Elevator Limit Switch REV", elevatorMotor->GetSensorCollection().IsRevLimitSwitchClosed());
}

int Elevator::GetElevatorEncoderPosition() {
	return elevatorMotor->GetSelectedSensorPosition(0);

}

double Elevator::GetElevatorMotorCurrent() {
	return elevatorMotor->GetOutputCurrent();
}