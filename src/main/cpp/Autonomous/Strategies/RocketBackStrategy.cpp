#include "Autonomous/Strategies/RocketBackStrategy.h"
#include <iostream>

#include "Subsystems/IntakeRotate.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/RotateIntake.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/DriveToTarget.h"


void RocketBackStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "RocketBackStrategy::Init\n";

    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;

    // Drive off platform
    const double initialDriveSpeed = 0.2;
    steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, initialDriveSpeed, 0.0, 1.0, 0.5),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));

    // Drive to Rocket
    const double rocketAngle = 150.0 * inv;
    const double rocketSpeed = 0.3;
    const double rocketX = 60;
    const double rocketY = 90;
    auto ctrlDrive = new ClosedLoopDrive2(rocketAngle, rocketSpeed, rocketX, rocketY, -1, DriveUnit::kInches, 5.0, 0.5, 12);
    steps.push_back(ctrlDrive);
    steps.push_back(new ConcurrentStep({
        new DriveToTarget(rocketAngle, 0.3, 5.0, 3.0)
    }));
    

}
