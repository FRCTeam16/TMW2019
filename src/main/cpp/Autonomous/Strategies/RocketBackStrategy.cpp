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
#include "Autonomous/Steps/AlignToTarget.h"
#include "Autonomous/Steps/Rotate.h"


void RocketBackStrategy::Init(std::shared_ptr<World> world) {
    std::cout << "RocketBackStrategy::Init\n";

    const AutoStartPosition startPosition = world->GetStartPosition();
	const bool isRight = AutoStartPosition::kRight == startPosition;
    const int inv = isRight ? 1 : -1;

    // start backwards
    const double startAngle = 180.0;
    steps.push_back(new SetGyroOffset(startAngle));

    // Drive off platform
    const double initialDriveSpeed = PrefUtil::getSet("Auto.RBS.initDriveSpeed", 0.3);
    steps.push_back(new ConcurrentStep({
		new TimedDrive(startAngle, initialDriveSpeed, 0.0, 1.25, 0.5),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));

    // Drive to Rocket
    const double rocketAngle = PrefUtil::getSet("Auto.RBS.rocketAngle", 150.0) * inv;
    const double rocketSpeedY = PrefUtil::getSet("Auto.RBS.rocketSpeedY", 0.4);
    const double rocketSpeedX = PrefUtil::getSet("Auto.RBS.rocketSpeedX", 0.226) * inv;
    const double rocketDistance = PrefUtil::getSet("Auto.RBS.rocketDistance", 195);
    const double rocketDistanceThreshold = PrefUtil::getSet("Auto.RBS.rocketDistThresh", 5);
    const double rocketRampUpTime = PrefUtil::getSet("Auto.RBS.rocketRampUpTime", 0.5);
    const double rocketRampDownDist = PrefUtil::getSet("Auto.RBS.rocketRampDownDist", 12);
    const double rocketTimeOut = PrefUtil::getSet("Auto.RBS.rocketTimeOut", 3.0);
    steps.push_back(new OpenDriveToDistance(rocketAngle, rocketSpeedY, rocketSpeedX, rocketDistance, 
                                            rocketDistanceThreshold, rocketRampUpTime, rocketRampDownDist, rocketTimeOut));

    // Align to target
    const double rocketAlignmentThreshold = PrefUtil::getSet("Auto.RBS.rocketAlignmentThreshold", 5.0);
    const double rocketAlignmentTime = PrefUtil::getSet("Auto.RBS.rocketAlignmentTime", 1.0);
    const int rocketAlignmentHoldScans = PrefUtil::getSetInt("Auto.RBS.rocketAlignmentScans", 4);
    steps.push_back(new ConcurrentStep({
        new AlignToTarget(rocketAngle, rocketAlignmentThreshold, rocketAlignmentTime, rocketAlignmentHoldScans),
        // new DriveToTarget(rocketAngle, 0.0, rocketAlignmentThreshold, rocketAlignmentTime),
        new SetVisionLight(true)
    }, true));  // hard halt on end

    // Drive in while lifting elevator, score, then push back
    const double pushBackSpeed = PrefUtil::getSet("Auto.RBS.pushBackSpeed", 0.15);  // Used for pushIn/Out
    const double rocketPushInTime = PrefUtil::getSet("Auto.RBS.rocketPushInTime", 1.5);
    const double rocketPushOutTime = PrefUtil::getSet("Auto.RBS.rocketPushOutTime", 1.0);
    steps.push_back(new ConcurrentStep({
        new TimedDrive(rocketAngle, pushBackSpeed, 0.0, rocketPushInTime, -1, false),
        new SetElevatorPosition(Elevator::ElevatorPosition::kLevel2, 0.5)
    }));
	steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kEjectHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(rocketAngle, -pushBackSpeed, 0.0, rocketPushOutTime, -1, false),
        new SetVisionLight(false)
    }));

    // Prepare to drive back to hatch pickup area
    const double pickupAngle = 180.0;
    const double kickoutSpeed = PrefUtil::getSet("Auto.RBS.rocketKickoutSpeed", -0.2) * inv;
    const double kickoutTime = PrefUtil::getSet("Auto.RBS.rocketKickoutTime", 0.75);
    steps.push_back(new TimedDrive(pickupAngle, 0.0, kickoutSpeed, kickoutTime, 0.25));  // kick out horizontally

    // Drive back to hatch pickup
    const double pickupFastSpeed = PrefUtil::getSet("Auto.RBS.pickupFastSpeed", -0.4);
    const double pickupFastTime1 = PrefUtil::getSet("Auto.RBS.pickupFastTime1", 2.0);
    const double pickupFastTime2 = PrefUtil::getSet("Auto.RBS.pickupFastTime2", 1.0);
    const double pickupFastSpeedX = PrefUtil::getSet("Auto.RBS.pickupFastSpeedX", 0.05) * inv;
    steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pickupFastSpeed, 0.0, pickupFastTime1, 0.5),
        new SetElevatorPosition(Elevator::ElevatorPosition::kFloor, 1.0)
    }));
    steps.push_back(new TimedDrive(pickupAngle, pickupFastSpeed, pickupFastSpeedX, pickupFastTime2));

    // Do hatch pickup
    const double pickupVizSpeed = PrefUtil::getSet("Auto.RBS.pickupVizSpeed", 0.25);
    const double pickupVizTime = PrefUtil::getSet("Auto.RBS.pickupVizTime", 5.0);
    const double pickupVizTimeout = PrefUtil::getSet("Auto.RBS.pickupVizTimeout", 2.5);
    steps.push_back(new ConcurrentStep({
        new DriveToTarget(pickupAngle, pickupVizSpeed, pickupVizTime, pickupVizTimeout),    // robot centric
        new SetVisionLight(true)
    }));

    // pickup hatch
    steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pushBackSpeed, 0.0, 0.5),
        new SetVisionLight(false)
    })); 
    //
    const double secondRocketAngle = 30.0 * inv;
    auto secondRocketRotate =new Rotate(secondRocketAngle);
    secondRocketRotate->SetContinueOnTimeout(true);
    steps.push_back (secondRocketRotate);
    
}
