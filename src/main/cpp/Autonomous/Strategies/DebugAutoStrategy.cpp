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
#include "Autonomous/Steps/SetVisionLight.h"
#include "Autonomous/Steps/StopAtTargetCount.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/AlignToTarget.h"


DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	// DebugStraight();
	// DebugAutoHalt();
	// DebugControlled();
	// auto drive = new TimedDrive(150.00, -0.15, 0.0, 1.5, -1, false);

	const bool isRight = AutoStartPosition::kRight == world->GetStartPosition();
	const int inv = isRight ? 1 : -1;

	// DebugTargetCount(world);
	std::cout << "--- DEBUG Autonomous Doing Nothing ---\n";
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

void DebugAutoStrategy::DebugTargetCount(std::shared_ptr<World> world) {
	const bool isRight = AutoStartPosition::kRight == world->GetStartPosition();
	const int inv = isRight ? 1 : -1;

	const double startAngle = -90.0;
    steps.push_back(new SetGyroOffset(startAngle));
	steps.push_back(new SetVisionLight(true));
	steps.push_back(new SelectVisionPipeline(isRight? 2 : 1));

	const double angle = -90.0 * inv;
	const double timeout = 8.0;

	auto drive = new TimedDrive(angle, 0.3, 0.0, timeout);
	steps.push_back(new StopAtTargetCount(drive, 3, true, 0.2, timeout));
	steps.push_back(new DriveToTarget(angle, 0.1, 5, 5, 8));
	// steps.push_back(new AlignToTarget(angle, ))
}

void DebugAutoStrategy::DebugRocketSecondMove() {
	// const bool isRight = AutoStartPosition::kRight == world->GetStartPosition();
	// const int inv = isRight ? 1 : -1;

	// // start backwards
    // const double startAngle = 180.0;
    // steps.push_back(new SetGyroOffset(startAngle));


	// const double angle = 30.0 * inv;
	// const double timeout = 8.0;
	// auto drive = new TimedDrive(angle, 0.4, -0.07 * inv, timeout, 1.0);
	// steps.push_back(new ConcurrentStep({
	// 	new StopAtTarget(drive, 5, 1, 1.0, timeout),
	// 	new SetVisionLight(true)
	// }));

	// auto driveTarget = new DriveToTarget(angle, 0.2, 5.0, 5.0, 8.0);
	// steps.push_back(driveTarget);
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