/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/ElevatorIntakeSetpointStrategy.h"
#include "Robot.h"
#include "Util/PrefUtil.h"

ElevatorIntakeSetpointStrategy::ElevatorIntakeSetpointStrategy() {
//  PrefUtil::getSet("ElevRotate.V.F", 100);
 PrefUtil::getSet("ElevRotate.V.L2", 100);
 PrefUtil::getSet("ElevRotate.V.L3", 100);
//  PrefUtil::getSet("ElevRotate.NV.F", 100);
 PrefUtil::getSet("ElevRotate.NV.L2", 100);
 PrefUtil::getSet("ElevRotate.NV.L3", 100);
}

double ElevatorIntakeSetpointStrategy::LookupElevatorSetpoint () {
    Elevator::ElevatorPosition elevatorPosition = Robot::elevator->GetElevatorPosition ();
    IntakeRotate::IntakePosition intakePosition = Robot::intakeRotate->GetIntakePosition ();


    if (IntakeRotate::IntakePosition::kLevelOne == intakePosition) {
        /* if (ElevatorPosition::kFloor == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.V.F", 100);
        } else  */
        
        if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.V.L2", 77000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.V.L3", 100000);
        }
    } else {
        /*if (ElevatorPosition::kFloor == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.NV.F", 100);
        } else */
        if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.NV.L2", 80000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.NV.L3", 103000);
        }
    }
}