/*
 * Delay.cpp
 *
 *  Created on: Mar 2, 2017
 *      Author: User
 */
#include <iostream>
#include <Autonomous/Steps/Delay.h>


bool Delay::Run(std::shared_ptr<World> world) {
	const double currentTime = world->GetClock();
	if (startTime < 0) {
		startTime = currentTime;
	}
    std::cout << "Delay::Current Time: " << currentTime << "\n";
	return ((currentTime - startTime) > delay);
}

