//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// Climber.h
// author: ssemtner
// created on: 2022-3-17
//==================================================

#ifndef SRC_SUBSYSTEMS_CLIMBER_H_
#define SRC_SUBSYSTEMS_CLIMBER_H_

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/CowTimer.h"

#include <iostream>

class Climber
{
private:
    CowLib::CowMotorController *m_LeftMotor;
    CowLib::CowMotorController *m_RightMotor;

    double m_LeftPosition;
    double m_RightPosition;

    int m_State = 0;
    void ClimberSM(void);

    enum CLIMB_STATE
    {
        NONE = 0,
        EXT_BOTH,
        CLIMB_MID,
        EXT_LEFT_MID,
        CLIMB_HIGH,
        EXT_RIGHT_HIGH,
        EXT_RIGHT_TRAV,
        CLIMB_TRAV,
    };

public:
    Climber(int leftMotor, int rightMotor);

    void SetLeftPosition(double position);
    void SetRightPosition(double position);

    double GetLeftPosition();
    double GetRightPosition();

    double GetLeftSetpoint() { return m_LeftPosition; }

    double GetRightSetpoint() { return m_RightPosition; }

    void NextState(void);
    void PrevState(void);

    // bool LeftAtTarget();
    // bool RightAtTarget();
    // bool AtTarget()
    // {
    //     return LeftAtTarget() && RightAtTarget();
    // }

    void ResetConstants();

    void handle();

    virtual ~Climber();
};

#endif /* SRC_SUBSYSTEMS_CLIMBER_H_ */