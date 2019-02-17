#pragma once
#include "Subsystems/IntakeRotate.h"

struct ElevatorMoveRequest {
	std::shared_ptr<IntakeRotate> intakeRotate;
	double startTime;
	double startSetpoint;
	double endSetpoint;
	IntakeRotate::IntakePosition currentPosition;
	bool requiredIntakeMove = false;
	bool sentIntakeRepositionMessage = false;
	bool sentIntakeReturnMessage = false;
	bool finished = false;
	double kTimeout = 5.0;

	ElevatorMoveRequest(std::shared_ptr<IntakeRotate> _intakeRotate, double _startSetpoint, double _targetSetpoint) {
		intakeRotate = _intakeRotate;
		startTime = frc::Timer::GetFPGATimestamp();
		startSetpoint = _startSetpoint;
		endSetpoint = _targetSetpoint;
		currentPosition = intakeRotate->GetIntakePosition();
	}

	double CalculateSetpoint() {
		if (finished) {
			return endSetpoint;
		}
		if ((frc::Timer::GetFPGATimestamp() - startTime) >= kTimeout) {
			std::cout << "!!! Elevator Move Request Timed Out, aborting move  and resetting to start position " << startSetpoint << "!!!\n";
			finished = true;
			endSetpoint = startSetpoint;
		}

		bool isVertical = IntakeRotate::IntakePosition::kLevelOne == intakeRotate->GetIntakePosition();
		bool inAllowedError = intakeRotate->InPosition();
		bool intakeInCorrectPosition = !isVertical && inAllowedError;

		if (intakeInCorrectPosition) {
			if (requiredIntakeMove && !sentIntakeReturnMessage) {
				intakeRotate->SetIntakePosition(currentPosition);	// return intake to original position
				sentIntakeReturnMessage = true;
			}
			return endSetpoint;
		} else {
			requiredIntakeMove = true;
			std::cout << "==> Holding Elevator until Intake is Rotated <==\n";
			if (!sentIntakeRepositionMessage) {
				if (isVertical) {
					// only send message if we are vertical, i.e. assume error was flagged moving to a safe position
					intakeRotate->SetIntakePosition(IntakeRotate::IntakePosition::kCargoShot);
				}
				sentIntakeRepositionMessage = true;
			}
			return startSetpoint;
		}
	} 
};