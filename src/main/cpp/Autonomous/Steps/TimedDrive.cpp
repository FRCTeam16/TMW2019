/*
 * TimedDrive.cpp
 *
 *  Created on: Feb 18, 2018
 *      Author: jsmith
 */

#include <Autonomous/Steps/TimedDrive.h>
#include <Robot.h>

bool TimedDrive::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
		Robot::driveBase->UseOpenLoopDrive();
		Robot::driveBase->SetTargetAngle(angle);
	}
	if ((currentTime - startTime) > timeToDrive) {
		Robot::driveBase->UseClosedLoopDrive();
		return true;
	} else {
		const double twist = (useTwist) ? Robot::driveBase->GetTwistControlOutput() : 0.0;
		crab->Update(
				(float) twist,
				(float) ySpeed,
				(float) xSpeed,
				true);
		return false;
	}
}
