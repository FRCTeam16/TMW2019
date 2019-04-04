#pragma once

#include <frc/Preferences.h>
#include "frc/Timer.h"
#include "Subsystems/Drive/SwerveWheel.h"
#include "Subsystems/Drive/DriveInfo.h"
#include "Util/RampUtil.h"
#include "Util/MovingAverageThreshold.h"
#include <memory>
#include <iostream>
#include <string>

class JackScrewControl {
public:
    explicit JackScrewControl(std::string name, std::shared_ptr<SwerveWheel> wheel) : name(name), wheel(wheel) {
        std::cout << "JackScrewControl(" << name << ")\n";
        currentState = JackScrewState::kSwerve;
    }
    void Init();
    enum class JackScrewState { kSwerve, kOpenLoop, kClosedLoop };
    enum class EndStateAction { kNone, kSwitchToControl, kSwitchToAmpDetect };

    void ConfigureControlled(double targetDistance, double controlSpeed, double controlTimeStart, EndStateAction action, bool _doRamp = false);  // only do ramping on first "move"
    void InitOpenLoop(double speed, EndStateAction action);
    void Run();
    void Hold();
    void HoldOpenAndFinish();

    double GetLastChange() { return lastChange; }
    double GetAccumulatedPosition() { return accumulatedPosition; }
    double GetControlSpeed() { return controlSpeed; }
    void SetControlSpeed(double speed) { controlSpeed = speed; }
    bool IsClosedLoop() { return currentState == JackScrewState::kClosedLoop; }
    double GetTargetDistance() { return startPosition + targetDistance; }
    JackScrewState GetCurrentState() { return currentState; }
    void SetCurrentState(JackScrewState newState) { 
        std::cout << "Setting JackScrewControl " << name << " to state " << static_cast<int>(newState) << "\n";
        currentState = newState; 
    }
    bool IsFinished() { return finished; }
    EndStateAction GetEndStateAction() { return endStateAction; }

private:
    const std::string name;
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

    double controlSpeed = 0.0;
    const double jackScrewRampTime = 0.25;
    double rotationCloseLoopThreshold = 7;
    double pullUpApproachThreshold = 7;
    double pullUpApproachSpeed = -0.10;
    bool firstThresholdRun = true;
    bool doRamp = false;

    const int kAmpSkipScanCount = 10;
    int currentAmpDelayScan = 0;

    EndStateAction endStateAction = EndStateAction::kNone;

    MovingAverageThreshold ampDetector{25, 3};
};