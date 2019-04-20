/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Subsystems/ElevatorIntakeSetpointStrategy.h"
#include "Robot.h"
#include "Util/BSPrefs.h"

ElevatorIntakeSetpointStrategy::ElevatorIntakeSetpointStrategy() {
//  BSPrefs::GetInstance()->GetDouble("ElevRotate.V.F", 100);
 BSPrefs::GetInstance()->GetDouble("Elevator.V.L2", 77000);
 BSPrefs::GetInstance()->GetDouble("Elevator.V.L3", 100000);
//  BSPrefs::GetInstance()->GetDouble("ElevRotate.NV.F", 100);
 BSPrefs::GetInstance()->GetDouble("Elevator.NV.L1", 57000);
 BSPrefs::GetInstance()->GetDouble("Elevator.NV.L2", 80000);
 BSPrefs::GetInstance()->GetDouble("Elevator.NV.L3", 103000);

 BSPrefs::GetInstance()->GetDouble("Elevator.cargoship.shot", 62500);
}

double ElevatorIntakeSetpointStrategy::LookupElevatorSetpoint () {
    Elevator::ElevatorPosition elevatorPosition = Robot::elevator->GetElevatorPosition ();
    IntakeRotate::IntakePosition intakePosition = Robot::intakeRotate->GetIntakePosition ();

    if (Elevator::ElevatorPosition::kSpecialCargo == elevatorPosition) {
        return BSPrefs::GetInstance()->GetDouble("Elevator.cargoship.shot", 62500);;
    } else if (IntakeRotate::IntakePosition::kLevelOne == intakePosition) {
        /* if (ElevatorPosition::kFloor == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("ElevRotate.V.F", 100);
        } else  */
        
        if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("Elevator.V.L2", 77000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("Elevator.V.L3", 100000);
        }
    } else {
        /*if (ElevatorPosition::kFloor == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("ElevRotate.NV.F", 100);
        } else */
        if (Elevator::ElevatorPosition::kLevel1 == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("Elevator.NV.L1", 57000); 
        } else if (Elevator::ElevatorPosition::kLevel2 == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("Elevator.NV.L2", 80000);
        } else if (Elevator::ElevatorPosition::kLevel3 == elevatorPosition) {
            return BSPrefs::GetInstance()->GetDouble("Elevator.NV.L3", 103000);
        }
    }
}