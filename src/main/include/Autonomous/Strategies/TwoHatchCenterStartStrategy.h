#pragma once

#include "Autonomous/Strategy.h"

class TwoHatchCenterStartStrategy : public StepStrategy {
 public:
  TwoHatchCenterStartStrategy(std::shared_ptr<World> world);
  ~TwoHatchCenterStartStrategy() = default;
  void Init(std::shared_ptr<World> world) override;
};
