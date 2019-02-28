
#include <Autonomous/Steps/PositionElevator.h>
#include <Robot.h>
#include "Autonomous/Steps/DelayParam.h"

bool PositionElevator::Run(std::shared_ptr<World> world) {
	const bool useDelay = DelayParam::DelayType::kNone != delayParam.delayType;
	std::cout << "PositionElevator::Run(useDelay? " << useDelay << " )\n";

	 if (firstRun) {
		firstRun = false;
		if (!useDelay) {
			std::cout << "Positioning Elevator\n";
			DoSetPosition();
		} else {
			if (DelayParam::DelayType::kTime == delayParam.delayType) {
				target = world->GetClock() + delayParam.value;
			} else {
				target = Robot::driveBase->GetDriveControlEncoderPosition() + delayParam.value;
			}
		}
	}

	bool targetHit = false;
	if (useDelay) {
		switch(delayParam.delayType) {
			case DelayParam::DelayType::kPosition:
				// FIXME: Only works with positives right now
				targetHit = Robot::driveBase->GetDriveControlEncoderPosition() > target;
				break;
			case DelayParam::DelayType::kTime:
			default:
				targetHit = world->GetClock() > target;
				break;
		}
	}

	if (targetHit) {
//		std::cout << "Delay Param target hit, requesting elevator position: " << position << "\n";
		DoSetPosition();
	}

	const bool positioned = (waitForPosition) ? Robot::elevator->InPosition() : true;
	return sentPosition && positioned;
}

void PositionElevator::DoSetPosition() {
	if (overrideElevatorPosition < 0) {
		Robot::elevator->SetElevatorPosition(position);
	} else {
		// Robot::elevator->SetElevatorSetpoint(overrideElevatorPosition);
		throw logic_error("PositionElevator->Elevator::SetElevatorSetpoint not visible");
	}
	sentPosition = true;
}
