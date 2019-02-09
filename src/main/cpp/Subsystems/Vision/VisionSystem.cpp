/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Vision/VisionSystem.h"

#include "Robot.h"
#include "Util/PrefUtil.h"
#include <iostream>


VisionSystem::VisionSystem() {
    limelight.reset(new Limelight());
    xoffsetController.reset(new XOffsetController(limelight));

    double P = PrefUtil::getSet("Vision.x.P", 0.5);
    double I = PrefUtil::getSet("Vision.x.I", 0.0);
    double D = PrefUtil::getSet("Vision.x.D", 0.0);
    double range = PrefUtil::getSet("Vision.x.range", 0.3);

    xoffPID.reset(
        new PIDController(P, I, D, xoffsetController.get(), xoffsetController.get() )
    );
    xoffPID->SetOutputRange(-range, range);
    xoffPID->SetSetpoint(0.0);
    xoffPID->Enable();
}

void VisionSystem::Run() {
    double xTranslate = 0.0;
    double y = 0.0;
    double x = 0.0;

    auto prefs = frc::Preferences::GetInstance();
    double P = prefs->GetDouble("Vision.x.P", 0.5);
    double I = prefs->GetDouble("Vision.x.I", 0.0);
    double D = prefs->GetDouble("Vision.x.D", 0.0);
    xoffPID->SetPID(P, I, D);

    SceneInfo scene = limelight->GetScene();
    if (scene.hasTarget) {
        xTranslate = xoffsetController->GetValue();
    }
    std::cout 
        << "hasTarget? " << scene.hasTarget
        << " | vision xtranslate: " << xTranslate << std::endl;

    auto crabInfo = new CrabInfo();
    crabInfo->xspeed = xTranslate;
    driveInfo.reset(crabInfo);
}

std::shared_ptr<CrabInfo> VisionSystem::GetLastDriveInfo() {
    return driveInfo;
}

void VisionSystem::ToggleCameraMode() {
    auto mode = limelight->ToggleCameraMode();
    std::cout << "Toggled to mode: " << static_cast<int>(mode) << std::endl;
}