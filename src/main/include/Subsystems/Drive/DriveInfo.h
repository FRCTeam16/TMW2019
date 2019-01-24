/*
 * DriveInfo.h
 *
 *  Created on: Mar 27, 2017
 *      Author: User
 */

#ifndef SRC_UTIL_DRIVEINFO_H_
#define SRC_UTIL_DRIVEINFO_H_

template <typename T>
struct DriveInfo {
	T FL = 0;
	T FR = 0;
	T RL = 0;
	T RR = 0;

	DriveInfo() {}
	DriveInfo(T value) : FL(value), FR(value), RL(value), RR(value) {}
};




#endif /* SRC_UTIL_DRIVEINFO_H_ */
