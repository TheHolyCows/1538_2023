//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.cpp
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#include "Claw.h"

Claw::Claw(int wristMotor, int intakeMotor, int solenoidChannel)
{
    m_WristMotor = new CowLib::CowMotorController(wristMotor);
    m_IntakeMotor = new CowLib::CowMotorController(intakeMotor);

    m_WristMotor->SetControlMode(CowLib::CowMotorController::POSITION);
    m_IntakeMotor->SetControlMode(CowLib::CowMotorController::SPEED);

    m_WristMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_IntakeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    m_WristMotor->GetInternalMotor()->SetSensorPhase(false);
    m_IntakeMotor->GetInternalMotor()->SetSensorPhase(false);

    m_Solenoid = new frc::Solenoid(frc::PneumaticsModuleType::CTREPCM, solenoidChannel);

    m_WristPosition = 0;
    m_IntakeSpeed = 0;

    m_Open = false;

    ResetConstants();
}

void Claw::SetWristPosition(double position)
{
    m_WristPosition = position;
}

double Claw::GetWristPosition()
{
    return m_WristMotor->GetPosition();
}

void Claw::SetIntakeSpeed(double speed)
{
    m_IntakeSpeed = speed;
}

void Claw::SetOpen(bool open)
{
    m_Open = open;
}

void Claw::ResetConstants()
{
    m_WristMotor->SetPIDGains(CONSTANT("CLAW_P"), CONSTANT("CLAW_I"), CONSTANT("CLAW_D"), CONSTANT("CLAW_F"), 1);
}

void Claw::Handle()
{
    if(m_IntakeMotor)
    {
        m_IntakeMotor->Set(m_IntakeSpeed);
    }

    if(m_WristMotor)
    {
        m_WristMotor->Set(m_WristPosition);
    }
}

Claw::~Claw()
{
    delete m_IntakeMotor;
    delete m_WristMotor;
    delete m_Solenoid;
}