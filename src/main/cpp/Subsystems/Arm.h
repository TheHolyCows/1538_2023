//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// Climber.h
// author: Goober Gang
// created on: 2023-1-14
//==================================================

#ifndef SRC_SUBSYSTEMS_ARM_H_
#define SRC_SUBSYSTEMS_ARM_H_

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/CowTimer.h"

#include <iostream>

class Arm
{
private:
    CowLib::CowMotorController *m_LeftMotor;
    CowLib::CowMotorController *m_RightMotor;

    double m_Position;

public:
    Arm(int leftMotor, int rightMotor);

    void SetPosition(double position);


    double GetPosition();

    void ResetConstants();

    void handle();

    virtual ~Arm();
};

#endif /* SRC_SUBSYSTEMS_ARM_H_ */