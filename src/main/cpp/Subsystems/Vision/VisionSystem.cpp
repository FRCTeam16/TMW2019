/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Vision/VisionSystem.h"

#include "Robot.h"
#include "Util/BSPrefs.h"
#include <iostream>


VisionSystem::VisionSystem() {
    limelight.reset(new Limelight());
    xoffsetController.reset(new XOffsetController(limelight));

    double P = BSPrefs::GetInstance()->GetDouble("Vision.x.P", 0.5);
    double I = BSPrefs::GetInstance()->GetDouble("Vision.x.I", 0.0);
    double D = BSPrefs::GetInstance()->GetDouble("Vision.x.D", 0.0);
    double xThreshold = BSPrefs::GetInstance()->GetDouble("Vision.x.threshold", 50.0);
    double range = BSPrefs::GetInstance()->GetDouble("Vision.x.range", 0.3);

    xoffPID.reset(
        new PIDController(P, I, D, xoffsetController.get(), xoffsetController.get() )
    );
    xoffPID->SetOutputRange(-range, range);
    xoffPID->SetSetpoint(0.0);
    xoffPID->Enable();
}

void VisionSystem::Run() {
    double y = 0.0;
    double x = 0.0;

    auto prefs = BSPrefs::GetInstance();
    double P = prefs->GetDouble("Vision.x.P", 0.5);
    double I = prefs->GetDouble("Vision.x.I", 0.0);
    double D = prefs->GetDouble("Vision.x.D", 0.0);
    double xThreshold = prefs->GetDouble("Vision.x.threshold", 50.0);
    xoffPID->SetPID(P, I, D);

    auto driveInfo = new VisionInfo();
    SceneInfo scene = limelight->GetScene();
    if (scene.hasTarget) {
        driveInfo->hasTarget = true;
        driveInfo->xSpeed = -xoffsetController->GetValue();
        driveInfo->xOffset = scene.xOffset;
        driveInfo->inThreshold = fabs(scene.xOffset) <= xThreshold;
        driveInfo->targetArea = scene.targetArea;
    }
    // std::cout 
    //     << "hasTarget? " << scene.hasTarget
    //     << " | vision xtranslate: " << xTranslate << std::endl;
    currentVisionInfo.reset(driveInfo);
}

std::shared_ptr<VisionInfo> VisionSystem::GetLastVisionInfo() {
    return currentVisionInfo;
}

void VisionSystem::ToggleCameraMode() {
    auto mode = limelight->ToggleCameraMode();
    std::cout << "Toggled to mode: " << static_cast<int>(mode) << std::endl;
}

void VisionSystem::Instrument() {
    if (currentVisionInfo) {
        frc::SmartDashboard::PutBoolean("Vision Target?", currentVisionInfo->hasTarget);
        frc::SmartDashboard::PutNumber("Vision Threshold?", currentVisionInfo->inThreshold);
    }
}

void VisionSystem::SetMaxOutputRange(double range) {
    std::cout << "Setting VisionSystem MaxOutputRange: " << range << "\n";
    xoffPID->SetOutputRange(-range, range);
}

void VisionSystem::ResetMaxOutputRange() {
    double range = BSPrefs::GetInstance()->GetDouble("Vision.x.range", 0.3);
    SetMaxOutputRange(range);
}