/*
 * DriveUnit.h
 *
 *  Created on: Feb 17, 2018
 *      Author: smithj11
 */

#ifndef SRC_AUTONOMOUS_DRIVEUNIT_H_
#define SRC_AUTONOMOUS_DRIVEUNIT_H_

class DriveUnit {
public:
	enum Units { kPulses, kInches };
	static double ToPulses(double value, Units units);
	static double ToInches(double value, Units units);
	static const std::string PULSES_PER_INCH;
};

#endif /* SRC_AUTONOMOUS_DRIVEUNIT_H_ */
