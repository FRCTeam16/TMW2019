/*
 * World.h
 *
 *  Created on: Feb 6, 2017
 *      Author: User
 */

#ifndef SRC_AUTONOMOUS_WORLD_H_
#define SRC_AUTONOMOUS_WORLD_H_

#include "frc/WPILib.h"
#include "FieldInfo.h"
#include "AutoPositions.h"

using namespace frc;


class AutoManager;

class World {
private:
	bool isRed;
	FieldInfo fieldInfo;
	AutoStartPosition startPosition;

	double driveDistance = 0;
	double driveDistanceOvershoot = 0;
	bool teamSideMode = false;
	bool autoTraverse = false;
public:
	World();
	virtual ~World() {}

	void Init();							// perform world initialization
	double GetClock() const;				// time elapsed since Init() in seconds
	bool IsRed();
	FieldInfo GetFieldInfo();
	void SetFieldInfo(FieldInfo fieldInfo);
	AutoStartPosition GetStartPosition() { return startPosition; }
	void SetStartPosition(AutoStartPosition pos) { startPosition = pos; }

	void SetDriveDistance(double d) { driveDistance = d; }
	double GetDriveDistance() { return driveDistance; }

	void SetDriveDistanceOvershoot(double d) { driveDistanceOvershoot = d; }
	double GetDriveDistanceOvershoot() { return driveDistanceOvershoot; }
};

#endif /* SRC_AUTONOMOUS_WORLD_H_ */
