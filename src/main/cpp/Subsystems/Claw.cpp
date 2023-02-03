//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.cpp
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#include "Claw.h"
#include "../CowLib/Conversions.h"

Claw::Claw(int wristMotor, int intakeMotor, int solenoidChannel)
{
    m_WristMotor = new CowLib::CowMotorController(wristMotor);
    m_IntakeMotor = new CowLib::CowMotorController(intakeMotor);

    m_WristMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_IntakeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    m_Solenoid = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, solenoidChannel);

    m_WristPosition = 0;
    m_IntakeSpeed = 0;

    m_Open = false;

    ResetConstants();
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
    m_IntakeControlRequest.PercentOut = percent * CONSTANT("ARM_INTAKE_RATIO");
}

void Claw::SetOpen(bool open)
{
    m_Open = open;
}

void Claw::ResetConstants()
{
    m_WristMotor->SetPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
}

void Claw::Handle()
{
    if(m_IntakeMotor)
    {
        m_IntakeMotor->Set(m_IntakeControlRequest);
    }

    if(m_WristMotor)
    {
        m_WristMotor->Set(m_WristControlRequest);
    }
}

Claw::~Claw()
{
    delete m_IntakeMotor;
    delete m_WristMotor;
    delete m_Solenoid;
}