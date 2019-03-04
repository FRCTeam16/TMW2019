#include "Autonomous/Strategies/DebugAutoStrategy.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/PositionElevator.h"
#include "Autonomous/Steps/ClosedLoopDrive2.h"
#include "Autonomous/Steps/DriveToBump.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/RotateUntilPast.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/RunIntakeWithDelay.h"

#include "Autonomous/Steps/VisionControlledDrive.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/RotateIntake.h"
#include "Autonomous/Steps/DoIntakeAction.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/OpenDriveToDistance.h"


DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	// DebugStraight();
	// DebugAutoHalt();
	// DebugControlled();
	auto drive = new TimedDrive(150.00, -0.15, 0.0, 1.5, -1, false);
	steps.push_back(drive);
}

void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	// const double angle = -180;
	// SetGyroOffset *step = new SetGyroOffset(angle);
	// step->Run(world);
}

void DebugAutoStrategy::DebugAutoHalt() {
	const double angle = 90.0;
	auto drive = new TimedDrive(angle, 0.2, 0.0, 5.0, 0.5);
	steps.push_back(new StopAtTarget(drive, 5, 5, 0.0, 5.0));
	// steps.push_back(new DriveToTarget(angle, 0.2, 5.0, 2.0));
}

void DebugAutoStrategy::DebugStraight() {


	// steps.push_back(new VisionControlledDrive(0.0, 0.15, 0, 60, -1, DriveUnit::kInches, 5.0, 0.5, 6));
	// steps.push_back(new VisionControlledDrive(0.0, 0.15, 0.0, 0.5));

	
	steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, 0.3, 0.0, 1.0),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));
	steps.push_back(new DriveToTarget(0.0, 0.3, 5.0, 3.5));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new TimedDrive(0.0, -0.15, 0.0, 0.5));

	// Move to hatch pickup
	const double lateralX = 0.3;
	const double lateralY = lateralX * 0.56;	// ratio of distances, FIXME for other side
	steps.push_back(new TimedDrive(-180.0, -lateralY, -lateralX, 2.5, 0.5));

	steps.push_back(new DriveToTarget(-180.0, 0.2, 5, 3.0));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new TimedDrive(-180.0, 0.15, 0.0, 0.5)); // FIXME no gyro robot centric

}

void DebugAutoStrategy::DebugControlled() {
	// auto drive = new ClosedLoopDrive2(0.0, 0.3, 36, 36, -1, DriveUnit::kInches, 3.0, 0.5, 6);
	auto drive = new OpenDriveToDistance(0.0, 0.3, 0.3, 36, 5, 0.6, 6, 4.0);
	steps.push_back(drive);
}