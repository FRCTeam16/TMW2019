/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Subsystems/SubsystemManager.h"

class RampSystem : public SubsystemManager {
public:
  RampSystem();

  void Run() override;

  void ToggleDeploy();
private:
  bool deployRequested = false;
  bool deployed = false;
};
