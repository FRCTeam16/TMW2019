#include "Autonomous/Strategies/RocketBackStrategy.h"
#include <iostream>

#include "Subsystems/IntakeRotate.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/RotateIntake.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/SetElevatorPosition.h"
#include "Autonomous/Steps/DoIntakeAction.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/OpenDriveToDistance.h"
#include "Autonomous/Steps/SetVisionLight.h"


void RocketBackStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "RocketBackStrategy::Init\n";

    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;

    // start backwards
    steps.push_back(new SetGyroOffset(180.0));

    // Drive off platform
    const double initialDriveSpeed = 0.3;
    steps.push_back(new ConcurrentStep({
		new TimedDrive(180.0, initialDriveSpeed, 0.0, 1.25, 0.5),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));

    // Drive to Rocket
    const double rocketAngle = 150.0 * inv;
    const double rocketSpeed = 0.3;
    const double rocketX = 60 * inv;
    const double rocketY = 90;
    // auto ctrlDrive = new ClosedLoopDrive2(rocketAngle, rocketSpeed, rocketX, rocketY, -1, DriveUnit::kInches, 5.0, 0.5, 12);
    // steps.push_back(ctrlDrive);
    steps.push_back(new OpenDriveToDistance(rocketAngle, 0.4, 0.226, 195, 5, 0.5, 12, 3.0));

    // Align
    steps.push_back(new ConcurrentStep({
        new DriveToTarget(rocketAngle, 0.0, 5.0, 1.0),
        new SetVisionLight(true)
    }));

    // Drive in while lifting elevator, score
    const double pushBackSpeed = 0.15;
    steps.push_back(new ConcurrentStep({
        new TimedDrive(rocketAngle, pushBackSpeed, 0.0, 1.5, -1, false),
        new SetElevatorPosition(Elevator::ElevatorPosition::kLevel2, 1.0)
    }));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(rocketAngle, -pushBackSpeed, 0.0, 1.0, -1, false),
        new SetVisionLight(false)
    }));

    // Align to back pickup
    const double pickupAngle = 180.0;
    steps.push_back(new TimedDrive(pickupAngle, 0.0, -0.2 * inv, 0.75, 0.25));  // kick out
    steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, -0.4, 0.0, 2.0, 0.5),
        new SetElevatorPosition(Elevator::ElevatorPosition::kFloor, 1.0)
    }));
    steps.push_back(new TimedDrive(pickupAngle, -0.6, 0.05 * inv, 1.0));
    steps.push_back(new ConcurrentStep({
        new DriveToTarget(pickupAngle, 0.25, 5.0, 2.5),    // robot centric
        new SetVisionLight(true)
    }));

    // pickup hatch then pop back
    steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pushBackSpeed, 0.0, 0.5),
        new SetVisionLight(false)
    }));
}
