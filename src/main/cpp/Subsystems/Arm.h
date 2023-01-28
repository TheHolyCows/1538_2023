//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../CowConstants.h"
#include "../CowLib/Conversions.h"
#include "../CowLib/CowMotorController.h"

#include <iostream>

class Arm
{
private:
    CowLib::CowMotorController *m_RotationMotor;
    CowLib::CowMotorController *m_TelescopeMotor;

    CowLib::CowMotorController::PositionPercentOutput m_RotationControlRequest{ 0 };
    CowLib::CowMotorController::PositionPercentOutput m_TelescopeControlRequest{ 0 };

    double m_TelescopePosition;
    double m_Angle;

    int m_LoopCount;

public:
    Arm(int rotationMotor, int telescopeMotor);

    void SetAngle(double angle);
    void SetTelescopePosition(double position);

    double GetAngle();
    double GetTelescopePosition();

    void ResetConstants();

    void Handle();

    virtual ~Arm();
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */