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
	const double elapsed = currentTime - startTime;
	if (elapsed > timeToDrive) {
		// FIXME: Robot::driveBase->UseClosedLoopDrive();
		return true;
	} else {
		const double twist = (useTwist) ? Robot::driveBase->GetTwistControlOutput() : 0.0;
		double y = ySpeed;
		if (rampUpTime > 0) {
			y = RampUtil::RampUp(ySpeed, elapsed, rampUpTime);
		}
		crab->Update(
				(float) twist,
				(float) y,
				(float) xSpeed,
				true);
		return false;
	}
}
