/*
 * Strategy.h
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_STRATEGY_H_
#define SRC_AUTONOMOUS_STRATEGY_H_

#include "frc/WPILib.h"
#include <vector>
#include "Step.h"

class World;

class Strategy {
public:
	Strategy() = default;
	virtual ~Strategy() = default;
	virtual void Init(std::shared_ptr<World> world) = 0;
	virtual bool Run(std::shared_ptr<World> world) = 0;
};

class StepStrategy : public Strategy {
public:
	StepStrategy() = default;
	virtual ~StepStrategy();
	virtual void Init(std::shared_ptr<World> world);
	virtual bool Run(std::shared_ptr<World> world);
protected:
	unsigned int currentStep = 0;
	std::vector<Step*> steps;
	void RunDrives(const CrabInfo *crab, bool showMessage = true);
	const std::unique_ptr<CrabInfo> STOP { new CrabInfo() };
private:
	bool finished = false;
};

#endif /* SRC_AUTONOMOUS_STRATEGY_H_ */
