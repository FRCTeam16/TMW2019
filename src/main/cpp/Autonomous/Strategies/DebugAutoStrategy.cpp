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


DebugAutoStrategy::DebugAutoStrategy(std::shared_ptr<World> world) {
	Debug();
}

void DebugAutoStrategy::Init(std::shared_ptr<World> world) {
	std::cout << "DebugAutoStrategy::Init()\n";
	AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;

	const int inv = isRight ? 1 : -1;
	const double angle = -180;
	SetGyroOffset *step = new SetGyroOffset(angle);
	step->Run(world);
}

void DebugAutoStrategy::Debug() {
	const double angle = -180.0;

	steps.push_back(new ConcurrentStep({
		new TimedDrive(angle, 0.0001, 0.0, 0.1, false),
		// new PositionMast(Mast::MastPosition::kDrive, DelayParam(DelayParam::DelayType::kNone, 0.0), true),
		// new IntakeSolenoidWithDelay(false, DelayParam(DelayParam::DelayType::kNone, 0.0), 5.0)
	}, true));


	const double firstDriveSpeed = 0.75;
	const double firstDriveX = 0.0;
	const double firstDriveY = 72.0;

	steps.push_back(new ConcurrentStep({
		new ClosedLoopDrive2(angle, firstDriveSpeed, firstDriveX, firstDriveY, -1, DriveUnit::Units::kInches, 8.0, 0.5, -1)
		// new PositionElevator(Elevator::ElevatorPosition::kSwitch, DelayParam(DelayParam::DelayType::kNone, 0.0)),
	}));

	// 	DriveToBump *bumpDrive = new DriveToBump(startAngle, ySpeed, xSpeed, timeout, 1.0, collisionThreshold );
	// 	bumpDrive->SetRampTime(0.5);


}
