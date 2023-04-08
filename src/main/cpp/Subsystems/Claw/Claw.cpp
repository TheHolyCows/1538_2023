//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Claw.cpp
// author: Cole/JT/Constantine
// created on: 2023-1-28
//==================================================

#include "Claw.h"

#include <frc/smartdashboard/SmartDashboard.h>

Claw::Claw(int wristMotor, int intakeMotor, int solenoidChannel)
{
    m_WristMotor  = new CowLib::CowMotorController(wristMotor);
    m_IntakeMotor = new CowLib::CowMotorController(intakeMotor);

    m_WristMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_IntakeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    // TODO: don't like the hardcoded 40 here, see if we can put it elsewhere
    m_Solenoid = new frc::Solenoid(40, frc::PneumaticsModuleType::CTREPCM, solenoidChannel);

    m_WristPosition = 0;
    m_IntakePercent = 0;

    m_Open = false;

    m_CurrentFilter = new CowLib::CowLPF(CONSTANT("INTAKE_CURRENT_LPF"));
    m_TorqueCurrent = 0;

    ResetConstants();
}

void Claw::RequestWristAngle(double angle)
{
    // frc::SmartDashboard::PutNumber("wrist req", angle);
    // frc::SmartDashboard::PutNumber(
    //     "wrist actual",
    //     CowLib::Conversions::FalconToDegrees(m_WristMotor->GetPosition(), CONSTANT("WRIST_GEAR_RATIO")) * -1);

    m_WristControlRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

double Claw::GetWristSetpoint()
{
    return CowLib::Conversions::FalconToDegrees(m_WristControlRequest.Position, CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

bool Claw::WristAtTarget()
{
    return fabs(GetWristSetpoint() - GetWristAngle() < CONSTANT("WRIST_TOLERANCE"));
}

double Claw::GetWristAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_WristMotor->GetPosition(), CONSTANT("WRIST_GEAR_RATIO")) * -1;
}

void Claw::SetIntakeSpeed(double percent)
{
    m_IntakeControlRequest.PercentOut = percent * -1;

    m_IntakePercent = percent * -1;
}

double Claw::GetIntakeSpeed()
{
    return m_IntakeMotor->GetVelocity() * -1;
}

void Claw::SetOpen(bool open)
{
    m_Open = open;
}

void Claw::ResetStowTimer()
{
    // 8 * 20 ms
    m_StowTimer = 6;
}

bool Claw::IsStalled()
{
    // seems viable
    // used to determine if we have possession of game piece
    if (m_StowTimer-- > 0)
    {
        return false;
    }

    return m_TorqueCurrent >= CONSTANT("INTAKE_CURRENT_TRIGGER") && fabs(GetIntakeSpeed() < 5);
}

void Claw::ResetConstants()
{
    m_WristMotor->SetPID(CONSTANT("WRIST_P"), CONSTANT("WRIST_I"), CONSTANT("WRIST_D"), CONSTANT("WRIST_F"));
    m_WristMotor->SetMotionMagic(CONSTANT("WRIST_V"), CONSTANT("WRIST_A"));
    // m_IntakeMotor->SetPID(CONSTANT("INTK_P"), CONSTANT("INTK_I"), CONSTANT("INTK_D"), CONSTANT("INTK_F"));

    m_CurrentFilter->UpdateBeta(CONSTANT("INTAKE_CURRENT_LPF"));
}

void Claw::Handle()
{
    if (m_IntakeMotor)
    {
        m_IntakeMotor->Set(m_IntakeControlRequest);
    }

    if (m_WristMotor)
    {
        m_WristMotor->Set(m_WristControlRequest);
    }
    if (m_Solenoid)
    {
        m_Solenoid->Set(m_Open);
    }

    // don't think this is the most efficient way of doing this
    // ideally you only check when we are intaking...
    m_TorqueCurrent = m_CurrentFilter->Calculate(fabs(m_IntakeMotor->GetTorqueCurrent()));
}

Claw::~Claw()
{
    delete m_IntakeMotor;
    delete m_WristMotor;
    delete m_Solenoid;
}

void Claw::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        m_WristMotor->SetNeutralMode(CowLib::CowMotorController::NeutralMode::BRAKE);
    }
    else
    {
        m_WristMotor->SetNeutralMode(CowLib::CowMotorController::NeutralMode::COAST);
    }
}

void Claw::CheckPCM()
{
    m_Solenoid->Get();
}
