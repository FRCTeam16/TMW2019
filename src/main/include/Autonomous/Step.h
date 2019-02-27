/*
 * Step.h
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_STEP_H_
#define SRC_AUTONOMOUS_STEP_H_

#include "frc/WPILib.h"
#include "World.h"
#include "Subsystems/Drive/CrabInfo.h"


class Step {
public:
	Step() = default;
	virtual ~Step() = default;
	virtual bool Run(std::shared_ptr<World> world) = 0;
	virtual const CrabInfo* GetCrabInfo() { return crab.get(); }
	bool IsManualDriveControl() const { return manualDriveControl; }
protected:
	std::unique_ptr<CrabInfo> crab { new CrabInfo() };
	bool manualDriveControl = false;
};

#endif /* SRC_AUTONOMOUS_STEP_H_ */
