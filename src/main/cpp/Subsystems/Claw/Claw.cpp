//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.cpp
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#include "Claw.h"

Claw::Claw(int wristMotor, int intakeMotor, int solenoidChannel)
{
    // m_WristMotor  = new CowLib::CowMotorController(wristMotor);
    m_IntakeMotor = new CowLib::CowMotorController(intakeMotor);

    // m_WristMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_IntakeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    // TODO: don't like the hardcoded 40 here, see if we can put it elsewhere
    m_Solenoid = new frc::Solenoid(40, frc::PneumaticsModuleType::CTREPCM, solenoidChannel);

    m_WristPosition = 0;
    m_IntakePercent = 0;

    m_Open = false;

    // ResetConstants();
}

void Claw::SetWristPosition(double position)
{
    m_WristControlRequest.Position = position * CONSTANT("ARM_WRIST_RATIO");
}

double Claw::GetWristPosition()
{
    return m_WristMotor->GetPosition();
}

void Claw::SetIntakeSpeed(double percent)
{
    if (m_Open)
    {
        m_IntakeControlRequest.PercentOut = percent * CONSTANT("ARM_INTAKE_CUBE");
    }
    else
    {
        m_IntakeControlRequest.PercentOut = percent * CONSTANT("ARM_INTAKE_CONE");
    }
    m_IntakePercent = percent;
}

double Claw::GetIntakeSpeed()
{
    return m_IntakeMotor->GetVelocity();
}

void Claw::SetOpen(bool open)
{
    m_Open = open;
}

void Claw::ResetConstants()
{
    // m_WristMotor->SetPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
    // m_IntakeMotor->SetPID(CONSTANT("INTK_P"), CONSTANT("INTK_I"), CONSTANT("INTK_D"), CONSTANT("INTK_F"));
}

void Claw::Handle()
{
    if (m_IntakeMotor)
    {
        m_IntakeMotor->Set(m_IntakeControlRequest);
    }

    // if (m_WristMotor)
    // {
    //     m_WristMotor->Set(m_WristControlRequest);
    // }
    if (m_Solenoid)
    {
        m_Solenoid->Set(m_Open);
    }
}

Claw::~Claw()
{
    delete m_IntakeMotor;
    delete m_WristMotor;
    delete m_Solenoid;
}