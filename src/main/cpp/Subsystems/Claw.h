//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.h
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#ifndef SRC_SUBSYSTEMS_CLAW_H_
#define SRC_SUBSYSTEMS_CLAW_H_

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"

#include <iostream>
#include <frc/Solenoid.h>

class Claw
{
private:
    CowLib::CowMotorController *m_WristMotor;
    CowLib::CowMotorController *m_IntakeMotor;
    frc::Solenoid *m_Solenoid;

    double m_WristPosition;
    double m_IntakeSpeed;

    bool m_Open;

public:
    Claw(int wristMotor, int intakeMotor, int solenoidChannel);

    void SetWristPosition(double position);

    double GetWristPosition();

    void SetIntakeSpeed(double speed);

    void SetOpen(bool open);

    void ResetConstants();

    void Handle();

    virtual ~Claw();
};

#endif /* SRC_SUBSYSTEMS_CLAW_H_ */