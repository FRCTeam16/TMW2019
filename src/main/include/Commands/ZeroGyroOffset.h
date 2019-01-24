/*
 * ZeroGyroOffset.h
 *
 *  Created on: Mar 3, 2017
 *      Author: User
 */

#ifndef SRC_COMMANDS_ZEROGYROOFFSET_H_
#define SRC_COMMANDS_ZEROGYROOFFSET_H_

#include "frc/commands/Subsystem.h"
#include <Robot.h>

class ZeroGyroOffset : public frc::Command {
public:
	ZeroGyroOffset();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif /* SRC_COMMANDS_ZEROGYROOFFSET_H_ */
