/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/Vision/VisionSystem.h"

#include "Robot.h"
#include <iostream>


VisionSystem::VisionSystem() {
    limelight.reset(new Limelight());
    rotate.reset(new RotateController(limelight));

    auto prefs = frc::Preferences::GetInstance();
    double P = prefs->GetDouble("VisionSystem.rotP", 1.0);
    double I = prefs->GetDouble("VisionSystem.rotI", 0.0);
    double D = prefs->GetDouble("VisionSystem.rotD", 0.0);
    prefs->PutDouble("VisionSystem.rotP", P);
    prefs->PutDouble("VisionSystem.rotI", I);
    prefs->PutDouble("VisionSystem.rotD", D);

    rotatePID.reset(
        new PIDController(P, I, D, rotate.get(), rotate.get() )
    );
    rotatePID->SetOutputRange(-0.3, 0.3);
    rotatePID->SetSetpoint(0.0);
    rotatePID->Enable();
}

void VisionSystem::Run() {
    double twist = 0.0;
    double y = 0.0;
    double x = 0.0;

    auto prefs = frc::Preferences::GetInstance();
    double P = prefs->GetDouble("VisionSystem.rotP", 1.0);
    double I = prefs->GetDouble("VisionSystem.rotI", 0.0);
    double D = prefs->GetDouble("VisionSystem.rotD", 0.0);

    rotatePID->SetPID(P, I, D);

    SceneInfo scene = limelight->GetScene();
    if (scene.hasTarget) {
        twist = rotate->GetValue();
    }
    std::cout 
        << "hasTarget? " << scene.hasTarget
        << " vision twist: " << twist << std::endl;

    double lockAngle = calculateLockAngle(RobotMap::gyro->GetYaw());
    Robot::driveBase->SetTargetAngle(lockAngle);

    double robotTwist = Robot::driveBase->GetCrabTwistOutput();
    Robot::driveBase->Crab(robotTwist, y, twist, false);
}

void VisionSystem::ToggleCameraMode() {
    auto mode = limelight->ToggleCameraMode();
    std::cout << "Toggled to mode: " << static_cast<int>(mode) << std::endl;
}

double VisionSystem::calculateLockAngle(double gyro_) {
   double answer = -1;
   bool isNegative = gyro_ < 0;
   double gyro = fabs(gyro_);
   // put logic here
   if (gyro >= -30.0 && gyro < 30.0) {
       answer = 0.0;
   } else if (gyro  >= 30 && gyro < 75) {
       answer = 60,0;
   } else if (gyro >= 75 && gyro < 120) {
       answer = 90.0;
   } else if (gyro >= 120 && gyro < 165) {
       answer = 150.0;
   } else if (gyro >= 165 && gyro < 195) {
       answer = 180.0;
   // post-180 positive variables
   } else if (gyro >= 195 && gyro < 240) {
       answer = 210.0;
   } else if (gyro >= 240 && gyro < 285) {
       answer = 270.0;
   } else if (gyro >= 285 && gyro < 315) {
       answer = 300.0;
   } else if (gyro >= 315 && gyro <= 360) {
       answer = 360;
   } 
   
   int mult = 1;
   if (isNegative == true) {
       mult = -1;
   }
   
   return mult*answer;
   
}