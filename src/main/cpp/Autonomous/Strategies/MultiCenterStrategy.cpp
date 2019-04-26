#include "Autonomous/Strategies/MultiCenterStrategy.h"

#include <iostream>
#include "Util/BSPrefs.h"
#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/DoIntakeAction.h"
#include "Autonomous/Steps/DriveToTarget.h"
#include "Autonomous/Steps/RotateIntake.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/SetVisionLight.h"
#include "Autonomous/Steps/StopAtTargetCount.h"
#include "Autonomous/Steps/TimedDrive.h"
#include "Autonomous/Steps/SetVisionOutputRange.h"

void MultiCenterStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "MultiCenterStrategy::Init\n";

    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;

    const double pushBackSpeed = BSPrefs::GetInstance()->GetDouble("Auto.MCS.PushBack.y", -0.15);


    // Drive off platform
    const double initialDriveSpeedY = BSPrefs::GetInstance()->GetDouble("Auto.MCS.initDriveSpeed.y", 0.3);
    const double initialDriveSpeedX = BSPrefs::GetInstance()->GetDouble("Auto.MCS.initDriveSpeed.x", 0.0) * inv;
    const double visionOutputRange = BSPrefs::GetInstance()->GetDouble("Auto.MCS.visionOutputRange", 0.2);
    steps.push_back(new ConcurrentStep({
		new TimedDrive(0.0, initialDriveSpeedY, initialDriveSpeedX, 1.25, 0.25),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne),
        new SetVisionOutputRange(visionOutputRange),
        new SelectVisionPipeline(isRight? 2 : 1)
	}));

    // Drive to cargoship
    const double cargoShipAngle = -90.0 * inv;
    const double cargoDrive1Y = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive1.y", 0.4);
    const double cargoDrive1X = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive1.x", 0.1) * inv;
    const double cargoDrive1Time = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive1.time", 1.5);
    steps.push_back(new TimedDrive(cargoShipAngle, cargoDrive1Y, cargoDrive1X, cargoDrive1Time, -1));
    steps.push_back(new SetVisionLight(true));

    // Drive and Stop at Target
    const double cargoDrive2Y = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive2.y", 0.3);
    const double cargoDrive2Time = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive2.time", 5.0);
    const double cargoDrive2IgnoreTime = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive2.ignoretime", 0.0);
    const int cargoDrive2Target = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive2.tgt", 3);
    auto cargoDrive2 = new TimedDrive(cargoShipAngle, cargoDrive2Y, 0.0, cargoDrive2Time);
	steps.push_back(new StopAtTargetCount(cargoDrive2, cargoDrive2Target, isRight, cargoDrive2IgnoreTime, cargoDrive2Time));

    // Drive and Place Hatch
    DriveAndPlaceHatch(cargoShipAngle, inv);

    // Drive quickly to hatch pickup
    const double pickupAngle = 180.0;

    const double pickupFastSpeed = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupFastSpeed.y", -0.4);
    const double pickupFastSpeedX = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupFastSpeed.x", 0.2) * inv;
    const double pickupFastSpeedRamp = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupFastSpeedRamp", 0.5);
    const double pickupFastTime1 = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupFastTime1", 3.0);
    steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pickupFastSpeed, pickupFastSpeedX, pickupFastTime1),
        new SelectVisionPipeline(0),
        new SetVisionLight(true)
    }));


    // Pickup hatch
    const double pickupVizSpeed1 = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupVizSpeed1", 0.4);
    const double pickupVizArea1 = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupVizArea1", 2.0);
    const double pickupVizSpeed2 = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupVizSpeed2", 0.25);
    const double pickupVizArea2 = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupVizArea2", 5.0);
    const double pickupVizTimeout = BSPrefs::GetInstance()->GetDouble("Auto.MCS.pickupVizTimeout", 2.5);

    steps.push_back(new DriveToTarget(pickupAngle, pickupVizSpeed1, pickupVizArea1, pickupVizTimeout));
    steps.push_back(new DriveToTarget(pickupAngle, pickupVizSpeed2, pickupVizArea2, pickupVizTimeout));

    steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, -pushBackSpeed, 0.0, 0.5),
        new SetVisionLight(false)
    })); 

    // -------------------------
    // Second placement
    // -------------------------

    // Drive to cargoship
    const double cargoDrive3Y = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive3.y", 0.6);
    const double cargoDrive3X = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive3.x", -0.3) * inv;
    const double cargoDrive3Time = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive3.time", 1.5);
    steps.push_back(new TimedDrive(cargoShipAngle, cargoDrive3Y, cargoDrive3X, cargoDrive3Time, 0.5));
    steps.push_back(new SetVisionLight(true));
    steps.push_back(new SelectVisionPipeline(isRight? 2 : 1));

    // Drive and Stop at Target
    const double cargoDrive4Y = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive4.y", 0.3);
    const double cargoDrive4Time = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive4.time", 5.0);
    const double cargoDrive4IgnoreTime = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive4.ignoretime", 0.0);
    const int cargoDrive4Target = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDrive4.tgt", 2);
    auto cargoDrive4 = new TimedDrive(cargoShipAngle, cargoDrive4Y, 0.0, cargoDrive4Time);
	steps.push_back(new StopAtTargetCount(cargoDrive4, cargoDrive4Target, isRight, cargoDrive4IgnoreTime, cargoDrive4Time));
    steps.push_back(new SelectVisionPipeline(0));

    // Drive and Place Hatch
    DriveAndPlaceHatch(cargoShipAngle, inv, false);

}

void MultiCenterStrategy::DriveAndPlaceHatch(double cargoShipAngle, int inv, bool doPlacement) {
    const double placeHatchY = BSPrefs::GetInstance()->GetDouble("Auto.MCS.placeHatchY", 0.3);
	const double placeHatchThreshold = BSPrefs::GetInstance()->GetDouble("Auto.MCS.placeHatchThreshold", 5.0);
	const double placeHatchTimeout = BSPrefs::GetInstance()->GetDouble("Auto.MCS.cargoDriveTimeout", 3.5);
    const double pushBackSpeed = BSPrefs::GetInstance()->GetDouble("Auto.MCS.PushBack.y", -0.15);

	steps.push_back(new DriveToTarget(cargoShipAngle, placeHatchY, placeHatchThreshold, placeHatchTimeout));

    if (doPlacement) {
        steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
        steps.push_back(new Delay(0.25));
        steps.push_back(new ConcurrentStep({
            new TimedDrive(cargoShipAngle, 0.0, -pushBackSpeed * inv, 0.5),
            new SetVisionLight(false)
	    }));
    }
	
}
