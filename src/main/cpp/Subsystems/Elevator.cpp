#include <iostream>
#include "Subsystems/Elevator.h"
#include "Util/PrefUtil.h"
#include "Subsystems/IntakeRotate.h"
#include "Robot.h"

Elevator::Elevator() {
	std::cout << "Elevator starting\n";

	elevatorMotor->SetNeutralMode(NeutralMode::Brake);
	elevatorMotor->ConfigPeakOutputForward(1);
	elevatorMotor->ConfigPeakOutputReverse(-1);
	elevatorMotor->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
	elevatorMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
	elevatorMotor->ConfigSetParameter(ParamEnum::eClearPositionOnLimitF, 1, 0, 0, 0);
	// elevatorMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_FeedbackConnector, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen);
	// elevatorMotor->ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 1, 0, 0, 0);
	elevatorMotor->ConfigContinuousCurrentLimit(39);
	elevatorMotor->ConfigPeakCurrentLimit(0);
	elevatorMotor->EnableCurrentLimit(true);
	elevatorMotor->SetSensorPhase(false);
	// elevatorMotor->SetInverted(true);

	elevatorMotor->ConfigReverseSoftLimitThreshold(PrefUtil::getSetInt("Elevator.Pos.MaxHeight", -110000));
	elevatorMotor->ConfigReverseSoftLimitEnable(true);

	followerMotor->SetNeutralMode(NeutralMode::Brake);
	followerMotor->ConfigPeakOutputForward(1);
	followerMotor->ConfigPeakOutputReverse(-1);
	followerMotor->ConfigContinuousCurrentLimit(39);
	followerMotor->ConfigPeakCurrentLimit(0);
	followerMotor->EnableCurrentLimit(true);

	// followerMotor->ConfigForwardLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, mainElevatorMotor->GetDeviceID());
	// followerMotor->ConfigReverseLimitSwitchSource(LimitSwitchSource::LimitSwitchSource_Deactivated, LimitSwitchNormal::LimitSwitchNormal_NormallyOpen, mainElevatorMotor->GetDeviceID());
	followerMotor->Set(ControlMode::Follower, elevatorMotor->GetDeviceID());

	std::cout << "Elevator() complete\n";
}

void Elevator::Init() {
	elevatorPositionThreshold = PrefUtil::getSetInt("Elevator.Pos.Threshold", 10);
	PrefUtil::getSet("Elevator.BumpUp", -50);
	SetInitialPosition();

	runMode = kManual;
	initializeFinished = false;
	initializeScanCounts = 0;
}


void Elevator::SetInitialPosition() {
	runMode = RunMode::kManual;
	openLoopPercent = 0.0;
	elevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
}


void Elevator::Run() {
	const double now = frc::Timer::GetFPGATimestamp();
	RunMode mode = runMode;

    if (!initializeFinished && (initializeScanCounts++ < kInitializeScanCountMax)) {
        std::cout << "Elevator Overriding intitialization - current draw: " 
                  << elevatorMotor->GetOutputCurrent() << "\n";
        mode = kManual;
        openLoopPercent = 0.0;
    } else {
        if (initializeScanCounts > 0) {
            initializeFinished = true;
            initializeScanCounts = 0;
            mode = kMagic;
            setpoint = GetElevatorEncoderPosition();
        }   
    }

	// Check for homing signal
	auto sensors = elevatorMotor->GetSensorCollection();
	if (elevatorMotor->GetSensorCollection().IsFwdLimitSwitchClosed()) {
		if ((now - setpointStartMoveTime) > kFloorResetWaitPeriod) {
			setpointStartMoveTime = -1;
			if (ElevatorPosition::kFloor != elevatorPosition) {
				std::cout << "Elevator :: Reset to Floor Position to due RevLimitSwitch\n";
				elevatorPosition = ElevatorPosition::kFloor;
			}
		}
	}


	switch (mode) {
		case kManual:
			elevatorMotor->Set(ControlMode::PercentOutput, openLoopPercent);
			break;
		case kMagic:
			double P = PrefUtil::getSet("Elevator.P", 1);
			double F = PrefUtil::getSet("Elevator.F", 0.18);
			int V = PrefUtil::getSetInt("Elevator.V", 5592);
			int A = PrefUtil::getSetInt("Elevator.A", 5592);
			int iZone = PrefUtil::getSet("Elevator.izone", 0);
			double I = PrefUtil::getSet("Elevator.I", 0);


			elevatorMotor->ConfigMotionCruiseVelocity(V, 0);
			elevatorMotor->ConfigMotionAcceleration(A, 0);
			elevatorMotor->Config_kP(0, P, 0);
			elevatorMotor->Config_kI(0, 0, 0);
			elevatorMotor->Config_kD(0, 0, 0);
			elevatorMotor->Config_kF(0, F, 0);
			elevatorMotor->Config_IntegralZone(0, iZone);
			elevatorMotor->Config_kI(0, I);

			elevatorMotor->Set(ControlMode::MotionMagic, setpoint);
			break;
	}
}


void Elevator::SetOpenLoopPercent(double _openLoopPercent) {
	runMode = RunMode::kManual;
	if (_openLoopPercent > 0) {
		// Moving down
		openLoopPercent = _openLoopPercent * 0.3;
	} else {
		openLoopPercent = _openLoopPercent;
	}
}

void Elevator::DisabledZeroOutput() {
	elevatorMotor->Set(ControlMode::PercentOutput, 0.0);
}

Elevator::ElevatorPosition Elevator::GetElevatorPosition() {
	return elevatorPosition;
}

void Elevator::SetElevatorPosition(ElevatorPosition _elevatorPosition) {
	runMode = RunMode::kMagic;
	elevatorPosition = _elevatorPosition;
	switch(elevatorPosition) {
		case ElevatorPosition::kFloor:
			setpoint = PrefUtil::getSet("Elevator.pos.Floor", 4000);
			break;
		default:
			setpoint = elevatorSetpointStrategy.LookupElevatorSetpoint();
			
	}
	setpointStartMoveTime = frc::Timer::GetFPGATimestamp();
	std::cout << "Elevator::SetElevatorPosition(" << static_cast<int>(elevatorPosition) << ")| " << setpoint << "\n";
}

void Elevator::SetElevatorSetpoint(int _setpoint) {
	setpoint = _setpoint;
}


void Elevator::BumpUp() {
	if (RunMode::kMagic == runMode) {
		std::cout << "Elevator::BumpUp\n";
		const double bumpUpAmt = PrefUtil::getSet("Elevator.BumpUp", -50);
			setpoint = PrefUtil::getSet("Elevator.pos.Floor", 4000);
		setpointStartMoveTime = frc::Timer::GetFPGATimestamp();
		SetElevatorSetpoint(setpoint + bumpUpAmt);
	} else {
		std::cout << "Elevator::BumpUp - Unable to bump because we are in open loop\n";
	}
}

void Elevator::BumpDown() {
	if (RunMode::kMagic == runMode) {
		std::cout << "Elevator::BumpDown\n";
		const double bumpUpAmt = PrefUtil::getSet("Elevator.BumpUp", -50);
			setpoint = PrefUtil::getSet("Elevator.pos.Floor", 4000);
		setpointStartMoveTime = frc::Timer::GetFPGATimestamp();
		SetElevatorSetpoint(setpoint - bumpUpAmt);
	} else {
		std::cout << "Elevator::BumpDown - Unable to bump because we are in open loop\n";
	}
}

bool Elevator::InPosition() {
    double error = setpoint - GetElevatorEncoderPosition();
    bool inPosition = (abs(error) < elevatorPositionThreshold);
//	std::cout << "Elevator in position: " << inPosition << "\n";
	return inPosition;
}


void Elevator::IncreaseElevatorPosition() {
	std::cout << frc::Timer::GetFPGATimestamp() << " - Elevator::IncreaseElevatorPosition\n";
	if (ElevatorPosition::kSpecialCargo != elevatorPosition) { 
		int nextOrdinal = static_cast<int>(elevatorPosition)  + 1;
		if (nextOrdinal < ELEVATOR_POSITION_COUNT) {
			ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
			IntakeRotate::IntakePosition intakePosition = Robot::intakeRotate->GetIntakePosition();
			if (nextPosition == ElevatorPosition::kLevel1 && intakePosition == IntakeRotate::IntakePosition::kLevelOne){
				std::cout << "Skipping Level1\n";
				nextPosition = static_cast<ElevatorPosition>(nextOrdinal + 1);
			}

			SetElevatorPosition(nextPosition);
		}
	} else {
		SetElevatorPosition(ElevatorPosition::kFloor);
	}
}


void Elevator::DecreaseElevatorPosition() {
	std::cout << frc::Timer::GetFPGATimestamp() << " - Elevator::DecreaseElevatorPosition\n";
	if (ElevatorPosition::kSpecialCargo != elevatorPosition) {
		int nextOrdinal = static_cast<int>(elevatorPosition) - 1;
		if (nextOrdinal >= 0 ) {
			ElevatorPosition nextPosition = static_cast<ElevatorPosition>(nextOrdinal);
			IntakeRotate::IntakePosition intakePosition = Robot::intakeRotate->GetIntakePosition();
			if (nextPosition == ElevatorPosition::kLevel1 && intakePosition == IntakeRotate::IntakePosition::kLevelOne){
				nextPosition = static_cast<ElevatorPosition>(nextOrdinal - 1);
				std::cout << "Skipping Level1\n";
			}

			SetElevatorPosition(nextPosition);
		}
	} else {
		SetElevatorPosition(ElevatorPosition::kFloor);
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


void Elevator::ToggleCargoShotMode() {
	std::cout << "Elevator::ToggleCargoShotMode(" << cargoShotMode << " to " << !cargoShotMode << ")\n";
	cargoShotMode = !cargoShotMode;
	ElevatorPosition nextPosition = ElevatorPosition::kFloor;
	if (cargoShotMode) {
		nextPosition = ElevatorPosition::kSpecialCargo;
		Robot::intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kFloor);
	}
	SetElevatorPosition(nextPosition);
}

void Elevator::Instrument() {
	SmartDashboard::PutNumber("Elevator Position", GetElevatorEncoderPosition());
	SmartDashboard::PutNumber("Elevator Level", static_cast<int>(elevatorPosition));
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