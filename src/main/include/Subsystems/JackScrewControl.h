#pragma once

#include <frc/Preferences.h>
#include "frc/Timer.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "Util/RampUtil.h"
#include "Util/MovingAverageThreshold.h"
#include <memory>
#include <iostream>

class JackScrewControl {
public:
    explicit JackScrewControl(std::shared_ptr<SwerveWheel> wheel) : wheel(wheel) {
        std::cout << "********************* JackScrewControl *********************\n";
        currentState = JackScrewState::kSwerve;
    }
    enum class JackScrewState { kSwerve, kOpenLoop, kClosedLoop };
    enum class EndStateAction { kNone, kSwitchToControl, kSwitchToAmpDetect };

    void ConfigureControlled(double targetDistance, double controlTimeStart, EndStateAction action);
    void InitOpenLoop(double speed, EndStateAction action);
    void Run();
    void Hold();

    double GetLastChange() { return lastChange; }
    double GetAccumulatedPosition() { return accumulatedPosition; }
    double GetControlSpeed() { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    bool IsClosedLoop() { return currentState == JackScrewState::kClosedLoop; }
    double GetTargetDistance() { return startPosition + targetDistance; }
    JackScrewState GetCurrentState() { return currentState; }
    void SetCurrentState(JackScrewState newState) { 
        std::cout << "Setting JackScrewControl " << this << " to state " << static_cast<int>(newState) << "\n";
        currentState = newState; 
    }
    bool IsFinished() { return finished; }
    EndStateAction GetEndStateAction() { return endStateAction; }

private:
    const std::shared_ptr<SwerveWheel> wheel;
    JackScrewState currentState;
    double controlTimeStart = -1;
    double targetDistance = 0;
    double startPosition = 0;
    double lastPosition = 0;
    double currentPosition = 0;
    double lastChange = 0;
    double accumulatedPosition = 0;
    bool finished = false;

    double kJackScrewApproachSpeed = -0.10;
    double controlSpeed = 1.0;
    const double jackScrewRampTime = 0.1;
    const double rotationCloseLoopThreshold = 7;
    bool firstThresholdRun = true;

    EndStateAction endStateAction = EndStateAction::kNone;

    MovingAverageThreshold ampDetector{25, 3};
};