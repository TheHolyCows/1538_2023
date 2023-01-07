//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// Intake.h
// author: ssemtner
// created on: 2022-2-12
//==================================================

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include "../CowConstants.h"
#include "../CowLib/CowLPF.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/CowPID.h"

#include <frc/Solenoid.h>

class Intake
{
private:
    CowLib::CowMotorController *m_MotorIntake;
    CowLib::CowMotorController *m_MotorIndex;
    frc::Solenoid *m_Solenoid;
    double m_IntakeSpeed;
    double m_IndexSpeed;
    bool m_IntakeExtended;
    double m_Scale;

public:
    enum IntakeMode
    {
        INTAKE_OFF     = 0,
        INTAKE_EXHAUST = 1,
        INTAKE_SHOOT   = 2,
        INTAKE_INTAKE  = 3
    };

    Intake(int intakeMotor, int indexMotor, int solenoidChannelA, double scale);
    void SetSpeed(double intakeSpeed, double indexSpeed);
    void SetIntakeSpeed(double speed);
    void SetIndexSpeed(double speed);
    void SetExtended(bool extended);

    bool GetExtended() { return m_IntakeExtended; }

    void SetStatusFramePeriod(void);

    void handle();
    virtual ~Intake();
};

#endif /* SRC_SUBSYSTEMS_ARM_H */
