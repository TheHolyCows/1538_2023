//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Pivot.h"

#include "../../CowLib/CowLogger.h"

Pivot::Pivot(const int motorID)
{
    m_PivotMotor = std::make_shared<ctre::phoenix::motorcontrol::can::TalonFX>(motorID, "cowbus");
    m_PivotMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);

    m_TargetAngle = 0;
    m_TickCount   = 0;

    ResetConstants();
}

void Pivot::RequestAngle(double angle)
{
    m_TargetAngle = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("PIVOT_GEAR_RATIO"));
}

double Pivot::GetSetpoint()
{
    return CowLib::Conversions::FalconToDegrees(m_TargetAngle, CONSTANT("PIVOT_GEAR_RATIO"));
}

bool Pivot::AtTarget()
{
    return fabs(GetSetpoint() - GetAngle() < CONSTANT("PIVOT_TOLERANCE"));
}

double Pivot::GetAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetSelectedSensorPosition() / 2048,
                                                CONSTANT("PIVOT_GEAR_RATIO"));
}

void Pivot::UpdatePID(double armExt)
{
    double p
        = CONSTANT("PIVOT_P_BASE") + (CONSTANT("PIVOT_P_EXTENSION") * armExt) + (CONSTANT("PIVOT_P_ANGLE") * armExt);

    m_PivotMotor->Config_kP(0, p, 100);
    m_PivotMotor->Config_kI(0, CONSTANT("PIVOT_I"), 100);
    m_PivotMotor->Config_kD(0, CONSTANT("PIVOT_D"), 100);
    m_PivotMotor->Config_kF(0, CONSTANT("PIVOT_F"), 100);
}

void Pivot::ResetConstants()
{
    // rewrite
    m_PivotMotor->Config_kP(0, CONSTANT("PIVOT_P"), 100);
    m_PivotMotor->Config_kI(0, CONSTANT("PIVOT_I"), 100);
    m_PivotMotor->Config_kD(0, CONSTANT("PIVOT_D"), 100);
    m_PivotMotor->Config_kF(0, CONSTANT("PIVOT_F"), 100);
    m_PivotMotor->ConfigMotionAcceleration(CONSTANT("PIVOT_A"), 10);
    m_PivotMotor->ConfigMotionCruiseVelocity(CONSTANT("PIVOT_V"), 10);
}

void Pivot::Handle()
{
    m_PivotMotor->Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_TargetAngle * 2048);

    if (m_TickCount++ % 10 == 0) // 200 miliseconds
    {
        m_TickCount = 1;

        CowLib::CowLogger::LogMotor(9, 0, m_PivotMotor->GetSelectedSensorPosition() / 2048);
    }
}

void Pivot::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        m_PivotMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    }
    else
    {
        m_PivotMotor->SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    }
}
