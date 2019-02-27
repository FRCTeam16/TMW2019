/*
 * DebugAutoStrategy.h
 *
 *  Created on: Feb 9, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_
#define SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_

#include "../Strategy.h"
#include "frc/WPILib.h"


class DebugAutoStrategy : public StepStrategy {
public:
	DebugAutoStrategy(std::shared_ptr<World> world);
	virtual ~DebugAutoStrategy() {}
	void Init(std::shared_ptr<World> world) override;

private:
	void Debug();
};

#endif /* SRC_AUTONOMOUS_DEBUGAUTOSTRATEGY_H_ */
