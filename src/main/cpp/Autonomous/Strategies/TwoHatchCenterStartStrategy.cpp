#include "Autonomous/Strategies/TwoHatchCenterStartStrategy.h"
#include "networktables/NetworkTableInstance.h"
#include <iostream>

#include "Subsystems/IntakeRotate.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/DoIntakeAction.h"
#include "Autonomous/Steps/RotateIntake.h"


double putSet(NetworkTable *tbl, std::string key, double defaultValue) {
	if (tbl->ContainsKey(key)) {
		return tbl->GetNumber(key, defaultValue);
	} else {
		tbl->PutNumber(key, defaultValue);
		return defaultValue;
	}
}

TwoHatchCenterStartStrategy::TwoHatchCenterStartStrategy(std::shared_ptr<World> world) {
}

void TwoHatchCenterStartStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "TwoHatchCenterStartStrategy::Init\n";
    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;

    auto defaultInstance = nt::NetworkTableInstance::GetDefault();
    auto autoTbl = defaultInstance.GetTable("Auto")->GetTable("THCS");
	

    // Drive down ramp
    const double initialDriveSpeed = putSet(autoTbl.get(), "D1.y", 0.3);
    steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, initialDriveSpeed, 0.0, 1.0),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));

    // Drive and place hatch
	const double targetArea = putSet(autoTbl.get(), "DriveToTarget.TA", 5.0);
	const double pushBackSpeed = putSet(autoTbl.get(), "PushBack.y", -0.15);
	steps.push_back(new DriveToTarget(0.0, initialDriveSpeed, targetArea, 3.5));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new TimedDrive(0.0, pushBackSpeed, 0.0, 0.5));

	// Move to hatch pickup
	const double toPickupX = putSet(autoTbl.get(), "D2.x", 0.3) * inv;
	const double toPickupY = putSet(autoTbl.get(), "D2.y", -0.168);		// ratio is 0.56
	const double toPickupTime = putSet(autoTbl.get(), "D2.time", 2.5);
	steps.push_back(new TimedDrive(-180.0, toPickupY, toPickupX, toPickupTime, 0.5));

	// Do Hatch pickup
	const double d3PickupSpeed = putSet(autoTbl.get(), "D3.y", 0.2);
	steps.push_back(new DriveToTarget(-180.0, d3PickupSpeed, targetArea, 3.0));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new TimedDrive(-180.0, -pushBackSpeed, 0.0, 0.5));

	// Move to scoring position
	// Ratio 228 Y / 72 X 
	const double toCargoY = 0.3;
	const double toCargoX = 0.09;
	const double toCargoTime = 0.3;
	steps.push_back(new TimedDrive(90.0, toCargoY, toCargoX, toCargoTime));

}