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
#include "Autonomous/Steps/SelectVisionPipeline.h"
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
    const double initialDriveSpeed = BSPrefs::GetInstance()->GetDouble("Auto.RBS.initDriveSpeed", 0.3);
    steps.push_back(new ConcurrentStep({
		new TimedDrive(startAngle, initialDriveSpeed, 0.0, 1.25, 0.5),
		new RotateIntake(IntakeRotate::IntakePosition::kLevelOne)
	}));

    // Drive to Rocket
    const double rocketAngle = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketAngle", 150.0) * inv;
    const double rocketSpeedY = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketSpeedY", 0.4);
    const double rocketSpeedX = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketSpeedX", 0.226) * inv;
    const double rocketDistance = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketDistance", 195);
    const double rocketDistanceThreshold = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketDistThresh", 5);
    const double rocketRampUpTime = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketRampUpTime", 0.5);
    const double rocketRampDownDist = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketRampDownDist", 12);
    const double rocketTimeOut = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketTimeOut", 3.0);
    steps.push_back(new OpenDriveToDistance(rocketAngle, rocketSpeedY, rocketSpeedX, rocketDistance, 
                                            rocketDistanceThreshold, rocketRampUpTime, rocketRampDownDist, rocketTimeOut));

    // Align to target
    const double rocketAlignmentThreshold = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketAlignmentThreshold", 5.0);
    const double rocketAlignmentTime = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketAlignmentTime", 1.0);
    const int rocketAlignmentHoldScans = BSPrefs::GetInstance()->GetInt("Auto.RBS.rocketAlignmentScans", 4);
    steps.push_back(new ConcurrentStep({
        new AlignToTarget(rocketAngle, rocketAlignmentThreshold, rocketAlignmentTime, rocketAlignmentHoldScans),
        // new DriveToTarget(rocketAngle, 0.0, rocketAlignmentThreshold, rocketAlignmentTime),
        new SetVisionLight(true)
    }, true));  // hard halt on end

    // Drive in while lifting elevator, score, then push back
    const double pushBackSpeed = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pushBackSpeed", 0.15);  // Used for pushIn/Out
    const double rocketPushInTime = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketPushInTime", 1.5);
    const double rocketPushOutTime = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketPushOutTime", 1.0);
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
    const double kickoutSpeed = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketKickoutSpeed", -0.2) * inv;
    const double kickoutTime = BSPrefs::GetInstance()->GetDouble("Auto.RBS.rocketKickoutTime", 0.75);
    steps.push_back(new TimedDrive(pickupAngle, 0.0, kickoutSpeed, kickoutTime, 0.25));  // kick out horizontally

    // Drive back to hatch pickup
    const double pickupFastSpeed = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupFastSpeed", -0.4);
    const double pickupFastTime1 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupFastTime1", 2.0);
    const double pickupFastTime2 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupFastTime2", 1.0);
    const double pickupFastSpeedX = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupFastSpeedX", 0.05) * inv;
    steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pickupFastSpeed, 0.0, pickupFastTime1, 0.5),
        new SetElevatorPosition(Elevator::ElevatorPosition::kFloor, pickupFastTime1)
    }));
    steps.push_back(new TimedDrive(pickupAngle, pickupFastSpeed, pickupFastSpeedX, pickupFastTime2));

    // Do hatch pickup
    const double pickupVizSpeed1 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupVizSpeed1", 0.4);
    const double pickupVizSpeed2 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupVizSpeed2", 0.25);
    const double pickupVizArea1 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupVizArea1", 2.0);
    const double pickupVizArea2 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupVizArea2", 5.0);
    const double pickupVizTimeout = BSPrefs::GetInstance()->GetDouble("Auto.RBS.pickupVizTimeout", 2.5);
    steps.push_back(new ConcurrentStep({
        new DriveToTarget(pickupAngle, pickupVizSpeed1, pickupVizArea1, pickupVizTimeout),    // robot centric
        new SetVisionLight(true)
    }));
    steps.push_back(new DriveToTarget(pickupAngle, pickupVizSpeed2, pickupVizArea2, pickupVizTimeout));

    // pickup hatch
    steps.push_back(new DoIntakeAction(DoIntakeAction::Action::kIntakeHatch, 0.5));
	steps.push_back(new ConcurrentStep({
        new TimedDrive(pickupAngle, pushBackSpeed, 0.0, 0.5),
        new SetVisionLight(false)
    })); 


    // -------------------------
    // Second placement
    // -------------------------

    const double secondRocketAngle = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketAngle", 30.0) * inv;
    const double secondRocketY = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketY", 0.4);
    const double secondRocketX = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketX", -0.07) * inv;
    const double secondRocketTimeout = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketTimeout", 8.0);
    const double secondRocketRamp = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketRamp", 1.0);


    // Spin and align to rocket
	steps.push_back(new ConcurrentStep({
        new TimedDrive(secondRocketAngle, secondRocketY, secondRocketX, secondRocketTimeout, secondRocketRamp),
        new SetVisionLight(true),
        new SelectVisionPipeline(isRight? 2 : 1)
    }));

    // Use Vision to Drive to Target
    const double secondRocketDriveToTargetVizMin1 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketVizMin1", 2.0);
    const double secondRocketDriveToTargetVizMin2 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketVizMin2", 4.5);
    const double secondRocketDriveToTargetVizMax = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketDriveVizMax", 8);
    const double secondRocketDriveToTargetY1 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketDriveTargetY1", 0.4);
    const double secondRocketDriveToTargetY2 = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketDriveTargetY2", 0.25);

    const double secondRocketDriveToTargetTimeout = BSPrefs::GetInstance()->GetDouble("Auto.RBS.secRocketDriveTimeout", 5.0);
	auto driveTarget = new DriveToTarget(secondRocketAngle, secondRocketDriveToTargetY1, 
                    secondRocketDriveToTargetVizMin1, secondRocketDriveToTargetTimeout, 
                    secondRocketDriveToTargetVizMax);

    auto driveTarget2 = new DriveToTarget(secondRocketAngle, secondRocketDriveToTargetY2, 
                    secondRocketDriveToTargetVizMin2, secondRocketDriveToTargetTimeout, 
                    secondRocketDriveToTargetVizMax);

    steps.push_back(driveTarget);
    steps.push_back(driveTarget2);

    steps.push_back(new SetElevatorPosition(Elevator::ElevatorPosition::kLevel2, 0.5));
    steps.push_back(new SetVisionLight(false));
}
