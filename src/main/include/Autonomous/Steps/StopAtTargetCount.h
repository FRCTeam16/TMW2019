#pragma once

#include "Autonomous/Step.h"

class StopAtTargetCount : public Step {
public:
  explicit StopAtTargetCount(Step *step, int numberOfTargets, bool movingRight, double ignoreTime, double timeOutTime);
  // ~StopAtTarget();
  bool Run(std::shared_ptr<World> world) override;
  virtual const CrabInfo* GetCrabInfo() override { return step->GetCrabInfo(); }
	bool IsManualDriveControl() const override { return step->IsManualDriveControl(); }
  void SetMinTx(double _minTx) { kMinTx = _minTx; }
private:
  const std::unique_ptr<Step> step;
  const int numberOfTargets;
  const bool movingRight;
  const double ignoreTime;
  const double timeOutTime;

  double startTime = -1;
  int targetsFound = 0;
  bool timedOut;

  double kMinTx = 20.0;           // we have to be within this range to meet threshold
  const double kXTargetJump = 5.0;      // jump amount to look for to see next target 
  const double kXDirectionCheck = -10.0; // amount to make sure we are past to avoid vision jumping to previous target

  double currentTx = 0.0;
  double lastTx = 0.0;

};
