#include "Util/RampUtil.h"
#include <iostream>



double RampUtil::RampUp(double value, double elapsedTime, double rampTime, double minSpeed) {
	 /*
		 /  |
	   /    |
	  -------
	 */
	double speed = value;
	if (elapsedTime < rampTime) {
		speed = (value / rampTime) * elapsedTime;
		std::cout << "RampUp: " << elapsedTime << " Profiled Speed: " << speed << "\n";
		if (speed < minSpeed) {
			speed = minSpeed;
		}
	} else {
		speed = value;
	}
	return speed;
}

double RampUtil::RampDown(double baseSpeed, double currentPosition, double target, double threshold, double minSpeed) {
	double speed = baseSpeed;
	double error = target - currentPosition;
	if (abs(error) < abs(threshold) ) {
		speed = baseSpeed * (error / threshold);
		std::cout << "RampDown Error: " << error << " Profiled Speed: " << speed << "\n";
		if (speed < minSpeed) {
			speed = minSpeed;
			std::cout << "RampDown - keeping minimum speed of " << speed;
		}
	}
	return speed;
}
