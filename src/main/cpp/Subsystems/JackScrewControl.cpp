#include <Robot.h>
#include "Subsystems/JackScrewControl.h"


void JackScrewControl::Init() {
    rotationCloseLoopThreshold = BSPrefs::GetInstance()->GetDouble("JackScrewControl.CloseLoopThreshold", 7);
    pullUpApproachThreshold = BSPrefs::GetInstance()->GetDouble("JackScrewControl.PullUpApproachThreshold", 15);
    pullUpApproachSpeed = BSPrefs::GetInstance()->GetDouble("JackScrewControl.PullUpApproachSpeed", -0.10);

    const double ampThreshold = BSPrefs::GetInstance()->GetDouble("JackScrewControl.PullUpAmps", 30);
    const unsigned int ampCounts = BSPrefs::GetInstance()->GetInt("JackScrewControl.PullUpAmpsCounts", 3);

    ampDetector = MovingAverageThreshold{ampThreshold, ampCounts};
}

void JackScrewControl::ConfigureControlled(double targetDistance_, double controlSpeed_, double controlTimeStart_, EndStateAction action, bool _doRamp) {
    std::cout << "JackScrewControl::Init() " << name << "\n";
    targetDistance = targetDistance_;
    controlTimeStart = controlTimeStart_;
    startPosition = currentPosition = lastPosition = wheel->GetDriveEncoderPosition();
    accumulatedPosition = 0.0;
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = controlSpeed_;
    endStateAction = action;
    firstThresholdRun = true;
    finished = false;
    ampDetector.Reset();
    doRamp = _doRamp;
}

void JackScrewControl::InitOpenLoop(double speed, EndStateAction action) {
    SetCurrentState(JackScrewState::kOpenLoop);
    controlSpeed = speed;
    endStateAction = action;
    doRamp = false;
}

void JackScrewControl::Run() {
    if (JackScrewState::kSwerve == GetCurrentState()) {
        return; // not controllable by us
    }

    const double now = frc::Timer::GetFPGATimestamp();
    double elapsed = now - controlTimeStart;
    double speed = controlSpeed;
    
    // if (doRamp) {
    //     if (elapsed < 0.25) {
    //         const double kMinSpeed = 0.10;
    //         std::cout << "JSC " << name << "Running constant speed " << kMinSpeed << "\n";
    //         speed = kMinSpeed;
    //     }
    // }
    //     } else if (elapsed <= (jackScrewRampTime + 0.25)) {
    //         speed = RampUtil::RampUp(controlSpeed, elapsed - 0.25, jackScrewRampTime, kMinSpeed);
    //         std::cout << "JSC " << name << " ramped to " << speed << "\n";
    //     }
    // }
    
    currentPosition = wheel->GetDriveEncoderPosition();
    lastChange = (currentPosition - lastPosition);  // removed fabs
    accumulatedPosition += lastChange;
    const double remaining = (targetDistance - accumulatedPosition);    // removed fabs

    const double fRemain = fabs(remaining);
    const bool inApproachThreshold = fRemain < pullUpApproachThreshold;
    const bool inCloseLoopThreshold = fRemain < rotationCloseLoopThreshold;
    std::cout << "JSC[" << name << "] fRemain = " << fRemain << " | inApproachT? " << inApproachThreshold << " | inCloseLoopT? " << inCloseLoopThreshold << "\n";

    switch (endStateAction) {
        case EndStateAction::kNone:
            break;

        // Only when running amp
        case EndStateAction::kSwitchToAmpDetect:
            
            if (!finished && inApproachThreshold) {
                SetControlSpeed(pullUpApproachSpeed);
                speed = pullUpApproachSpeed;

                if (firstThresholdRun) {
                    // Configure Open Loop drops JackScrew direction to kNone
                    // so JSC takes over the final driving bit with switch to amp detect
                    Robot::jackScrews->ConfigureOpenLoop(pullUpApproachSpeed, EndStateAction::kSwitchToAmpDetect); // calls back and modifies our state
                    firstThresholdRun = false;
                    currentAmpDelayScan = 0;
                }
            
                if (currentAmpDelayScan++ >= kAmpSkipScanCount) {
                    if (!ampDetector.Check()) {
                        ampDetector.AddValue(wheel->GetDriveOutputCurrent());
                    } else {
                        std::cout << "JackScrewControl " << name << " detected amp spike\n";
                        SetControlSpeed(0.0);
                        speed = 0.0;
                        finished = true;
                    }
                }
            } // otherwise use jackscrew settings sent to us
            break;

        // Used when running down
        case EndStateAction::kSwitchToControl:
            
            if (!IsClosedLoop() && inCloseLoopThreshold) {
                std::cout << " JackScrewControl " << name << " flipping to closed loop\n";
                wheel->SetDriveSoftMinMaxOutput(-1.0, 1.0);
                SetCurrentState(JackScrewState::kClosedLoop);
                finished = true;
            }
            break;
    }

    switch (GetCurrentState()) {
        case JackScrewState::kOpenLoop:
            wheel->UseOpenLoopDrive(speed);
            break;
        case JackScrewState::kClosedLoop:
            wheel->UseClosedLoopDrive(GetTargetDistance());     // FIXME? does it only work positive direction
            break;
        default:
            std::cout << "!!! Warning : JackScrew " << name << " in swerve state but run() reaching control !!!\n";
    }
    lastPosition = currentPosition;
}

void JackScrewControl::Hold() {
    currentState = JackScrewState::kClosedLoop;
}

/**
 * Used to allow a single screw to finish and have the other enter a finished state
 */
void JackScrewControl::HoldOpenAndFinish() {
    SetControlSpeed(0.0);
    controlSpeed = 0.0;
    finished = true;
}