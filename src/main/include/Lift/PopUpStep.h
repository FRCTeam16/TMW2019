#pragma once

#include "Action.h"
#include "Subsystems/JackScrews.h"

class PopUpStep : public Action {
 public:
  PopUpStep();
  void Execute() override;
  bool IsFinished() override { return finished; }
 private:
  bool finished = false;
  bool firstAfterShifting = true;
};
