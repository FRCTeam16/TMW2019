#pragma once
#include "Autonomous/Strategy.h"

class MultiCenterStrategy : public StepStrategy {
 public:
  explicit MultiCenterStrategy(std::shared_ptr<World> world) {};
  void Init(std::shared_ptr<World> world) override;
  void DriveAndPlaceHatch(double cargoShipAngle, int inv, bool doPlacement=true);
};
