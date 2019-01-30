/*
 * RampUtil.h
 *
 *  Created on: Feb 17, 2018
 *      Author: jsmith
 */

#pragma once

class RampUtil {
public:
	RampUtil() {}
    virtual ~RampUtil() {}
    double RampUp(double value, double elapsedTime, double rampTime, double minSpeed = 0.10);
    double RampDown(double baseSpeed, double currentPosition, double target, double threshold, double minSpeed = 0.10);
};
