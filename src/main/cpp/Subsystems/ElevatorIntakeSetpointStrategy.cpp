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
 PrefUtil::getSet("Elevator.V.L2", 77000);
 PrefUtil::getSet("Elevator.V.L3", 100000);
//  PrefUtil::getSet("ElevRotate.NV.F", 100);
 PrefUtil::getSet("Elevator.NV.L1", 57000);
 PrefUtil::getSet("Elevator.NV.L2", 80000);
 PrefUtil::getSet("Elevator.NV.L3", 103000);
}

double ElevatorIntakeSetpointStrategy::LookupElevatorSetpoint () {
    Elevator::ElevatorPosition elevatorPosition = Robot::elevator->GetElevatorPosition ();
    IntakeRotate::IntakePosition intakePosition = Robot::intakeRotate->GetIntakePosition ();


    if (IntakeRotate::IntakePosition::kLevelOne == intakePosition) {
        /* if (ElevatorPosition::kFloor == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.V.F", 100);
        } else  */
        
        if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return PrefUtil::getSet("Elevator.V.L2", 77000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return PrefUtil::getSet("Elevator.V.L3", 100000);
        }
    } else {
        /*if (ElevatorPosition::kFloor == elevatorPosition) {
            return PrefUtil::getSet("ElevRotate.NV.F", 100);
        } else */
        if (Elevator::ElevatorPosition::kLevel1 == elevatorPosition) {
            return PrefUtil::getSet("Elevator.NV.L1", 57000); 
        } else if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return PrefUtil::getSet("Elevator.NV.L2", 80000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return PrefUtil::getSet("Elevator.NV.L3", 103000);
        }
    }
}