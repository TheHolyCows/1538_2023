//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.h
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#ifndef SRC_SUBSYSTEMS_CLAW_H_
#define SRC_SUBSYSTEMS_CLAW_H_

#include "../../CowConstants.h"
#include "../../CowLib/Conversions.h"
#include "../../CowLib/CowMotorController.h"

#include <frc/Solenoid.h>
#include <iostream>

class Claw
{
private:
    CowLib::CowMotorController *m_WristMotor;
    CowLib::CowMotorController *m_IntakeMotor;

    CowLib::CowMotorController::MotionMagicPercentOutput m_WristControlRequest{ 0 };
    CowLib::CowMotorController::PercentOutput m_IntakeControlRequest{ 0 };
    frc::Solenoid *m_Solenoid;

    double m_WristPosition;
    double m_IntakePercent;

    bool m_Open;

public:
    Claw(int wristMotor, int intakeMotor, int solenoidChannel);

    void RequestWristAngle(double position);

    double GetWristSetpoint();

    bool WristAtTarget();

    double GetWristAngle();

    void SetIntakeSpeed(double percent);

    double GetIntakeSpeed();

    void SetOpen(bool open);

    void ResetConstants();

    void Handle();

    virtual ~Claw();
};

#endif /* SRC_SUBSYSTEMS_CLAW_H_ */