//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"

#include <iostream>

class Arm
{
private:
    CowLib::CowMotorController *m_RotatorMotor;
    CowLib::CowMotorController *m_TelescopeMotor;

    CowLib::CowMotorController::PositionPercentOutput m_RotatorController;
    CowLib::CowMotorController::PositionPercentOutput m_TelescopeController;

    double m_Position;

public:
    Arm(int rotatorMotor, int telescopeMotor);

    void SetRotatorPos(double position);
    void SetTelescopePos(double position);

    double GetRotatorPos();
    double GetTelescopePos();

    void ResetConstants();

    void Handle();

    virtual ~Arm();
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */