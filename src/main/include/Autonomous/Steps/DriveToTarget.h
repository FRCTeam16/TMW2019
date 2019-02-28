#pragma once

#include "Autonomous/Step.h"

class DriveToTarget : public Step {
public:
  DriveToTarget(double angle, double yspeed, double targetArea, double timeout);
  bool Run(std::shared_ptr<World> world) override;
private:
  double startTime = -1;
  const double angle;
  const double yspeed;
  const double targetArea;
  const double timeout;
};
