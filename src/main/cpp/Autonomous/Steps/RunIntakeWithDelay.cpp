#include "Autonomous/Steps/RunIntakeWithDelay.h"
#include "Robot.h"
#include "Autonomous/DriveUnit.h"

bool RunIntakeWithDelay::Run(std::shared_ptr<World> world) {
	std::cout << "RunIntakeWithDelay::Run()\n";
	if (firstRun) {
		std::cout << "RunIntakeWithDelay -> First Run\n";
		firstRun = false;
		if (DelayParam::DelayType::kTime == delayParam.delayType) {
			target = world->GetClock() + delayParam.value;
		} else {
			target = Robot::driveBase->GetDriveControlEncoderPosition()
					+ DriveUnit::ToPulses(delayParam.value, DriveUnit::kInches);
		}
		std::cout << "RunIntakeWithDelay Target = " << target << "\n";
	}
	const double currentTime = world->GetClock();

	if (startedTime < 0) {
		bool targetHit = false;
		switch(delayParam.delayType) {
			case DelayParam::DelayType::kPosition:
				targetHit = Robot::driveBase->GetDriveControlEncoderPosition() >= target;
				break;
			case DelayParam::DelayType::kTime:
				targetHit = currentTime >= target;
				break;
			default:
				std::cout << "! RunIntakeWithDelay Warning using default delay type : " << delayParam.delayType << "\n";
				targetHit = true;
		}
		if (targetHit) {
			startedTime = currentTime;
			ConfigIntake();

			// Exit if we're just turning on
			if (timeToRun <= 0) {
				return true;
			}
		}
	} else {
		if ((currentTime - startedTime) > timeToRun) {
			Robot::intake->Stop();
			return true;
		}
	}
	return false;
}

void RunIntakeWithDelay::ConfigIntake() {
	switch (state) {
	case IntakeHatch:
		Robot::intake->IntakeHatch();
		break;
	case EjectHatch:
		Robot::intake->EjectHatch();
		break;
	}
}

