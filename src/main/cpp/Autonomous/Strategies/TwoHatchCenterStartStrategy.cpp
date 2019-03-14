#include "Autonomous/Strategies/TwoHatchCenterStartStrategy.h"
#include "Util/PrefUtil.h"
#include <iostream>

#include "Subsystems/IntakeRotate.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/DoIntakeAction.h"
#include "Autonomous/Steps/RotateIntake.h"
#include "Autonomous/Steps/StopAtTarget.h"
#include "Autonomous/Steps/RotateUntilPast.h"
#include "Autonomous/Steps/SetVisionLight.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/Delay.h"


TwoHatchCenterStartStrategy::TwoHatchCenterStartStrategy(std::shared_ptr<World> world) {
}

void TwoHatchCenterStartStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "TwoHatchCenterStartStrategy::Init\n";
    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;
	
    // Drive down ramp
    const double initialDriveSpeed = PrefUtil::getSet("Auto.THCS.D1.y", 0.3);
    steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, initialDriveSpeed, 0.0, 1.0),
		new SetVisionLight(false),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne),
		new SelectVisionPipeline(isRight? 2 : 1)
	}));

    // Drive and place hatch
	const double targetArea = PrefUtil::getSet("Auto.THCS.DriveToTarget.TA", 5.0);
	const double pushBackSpeed = PrefUtil::getSet("Auto.THCS.PushBack.y", -0.15);
	steps.push_back(new SetVisionLight(true));
	steps.push_back(new DriveToTarget(0.0, initialDriveSpeed, targetArea, 3.5));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new Delay(0.5));
	steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, pushBackSpeed, 0.0, 0.5),
		new SetVisionLight(false)
	}));

	// Move to hatch pickup
	const double toPickupX = PrefUtil::getSet("Auto.THCS.D2.x", 0.3) * inv;
	const double toPickupY = PrefUtil::getSet("Auto.THCS.D2.y", -0.168);		// ratio is 0.56
	const double toIgnoreTime = PrefUtil::getSet("Auto.THCS.D2.ignoreTime", 2);
	const double toPickupTime = PrefUtil::getSet("Auto.THCS.D2.time", 2.5);

	// Pulse a turn
	steps.push_back(new RotateUntilPast(isRight, -180.0, 30.0 * inv));
	steps.push_back(new SelectVisionPipeline(0));

	auto d2drive = new TimedDrive(-180.0, toPickupY, toPickupX, toPickupTime, 0.5);
	steps.push_back(new ConcurrentStep({
		new StopAtTarget(d2drive, 5, 1, toIgnoreTime, toPickupTime),
		new SetVisionLight(true)
	}));

	// Do Hatch pickup
	const double d3PickupSpeed = PrefUtil::getSet("Auto.THCS.D3.y", 0.2);
	steps.push_back(new DriveToTarget(-180.0, d3PickupSpeed, targetArea, 3.0));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new ConcurrentStep({
		new TimedDrive(-180.0, -pushBackSpeed, 0.0, 0.5),
		new SetVisionLight(false)
	}));

	// Move to scoring position
	const double cargoShipAngle = -90.0 * inv;

	// Fast drive to near cargo ship (31%)
	const double nearCSY = PrefUtil::getSet("Auto.THCS.nearCSY", 0.6);
	const double nearCSX = PrefUtil::getSet("Auto.THCS.nearCSX", 0.186) * inv;
	const double nearCSDriveTime = PrefUtil::getSet("Auto.THCS.nearCSDriveTime", 2.25);	
	steps.push_back(new SelectVisionPipeline(isRight ? 1 : 2));
	steps.push_back(new TimedDrive(cargoShipAngle, nearCSY, nearCSX, nearCSDriveTime, 0.5));

	// Ratio 228 Y / 72 X 
	const double toCargoVizThresh = PrefUtil::getSet("Auto.THCS.toCargoVizThresh", 10);
	const double toCargoY = PrefUtil::getSet("Auto.THCS.toCargoY", 0.20);
	const double toCargoX = PrefUtil::getSet("Auto.THCS.toCargoX", -0.065) * inv;		
	const double toCargoTime = PrefUtil::getSet("Auto.THCS.toCargoTime", 2.75);
	const double toCargoIgnoreTime = PrefUtil::getSet("Auto.THCS.toCargoIgnoreTime", 0.5);

	// Approach cargo ship and look for first target
	auto drive = new TimedDrive(cargoShipAngle, toCargoY, toCargoX, toCargoTime);
	steps.push_back(new ConcurrentStep({
		new StopAtTarget(drive, toCargoVizThresh, 1, toCargoIgnoreTime, toCargoTime),
		new SetVisionLight(true)
	}, true));

	const double cargoDriveY = PrefUtil::getSet("Auto.THCS.cargoDriveY", 0.3);
	const double cargoDriveThreshold = PrefUtil::getSet("Auto.THCS.cargoDriveThreshold", 5.0);
	const double cargoDriveTimeout = PrefUtil::getSet("Auto.THCS.cargoDriveTimeout", 3.5);
	steps.push_back(new DriveToTarget(cargoShipAngle, cargoDriveY, cargoDriveThreshold, cargoDriveTimeout));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new ConcurrentStep({
		new TimedDrive(cargoShipAngle, 0.0, -pushBackSpeed * inv, 0.5),
		new SetVisionLight(false)
	}));
	steps.push_back(new SelectVisionPipeline(0));
}