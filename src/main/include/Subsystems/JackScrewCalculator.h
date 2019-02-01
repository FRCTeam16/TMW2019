#pragma once

#include "frc/Preferences.h"
#include "frc/Timer.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "Util/RampUtil.h"
#include <memory>
#include <iostream>

class JackScrewCalculator {
public:
    JackScrewCalculator(std::shared_ptr<SwerveWheel> wheel, int targetDistance, int controlTimeStart) : 
      wheel(wheel), 
      targetDistance(targetDistance),
      controlTimeStart(controlTimeStart) {
        startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    }

    void Run();
    void Hold();

    double GetLastChange() const { return lastChange; }
    double GetAccumulatedPosition() const { return accumulatedPosition; }
    double GetControlSpeed() const { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    bool IsFinished() const { return abs(currentPosition - targetDistance) <= finishedThreshold; }
    bool IsClosedLoop() const { return closedLoop; }

private:
    const double controlTimeStart;
    const std::shared_ptr<SwerveWheel> wheel;
    const double targetDistance;
    double startPosition = -1;
    double lastPosition = 0;
    double currentPosition = -1;
    double lastChange = 0;
    double accumulatedPosition = 0;
    bool closedLoop = false;

    double controlSpeed = 1.0;  // FIXME: Duplicated in jackscrews
    const double jackScrewRampTime = 0.1;
    const double rotationCloseLoopThreshold = 7;
    const double finishedThreshold = 1;
};