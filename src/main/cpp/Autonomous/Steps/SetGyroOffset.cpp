/*
 * SetGyroOffset.cpp
 *
 *  Created on: Mar 1, 2017
 *      Author: User
 */

#include <Autonomous/Steps/SetGyroOffset.h>

bool SetGyroOffset::Run(std::shared_ptr<World> world) {
	std::cout << "Setting gyro offset to " << offset << "\n";
	RobotMap::gyro->SetOffset(offset);
	return true;
}


