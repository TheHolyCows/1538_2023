//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Pivot.h"

Pivot::Pivot(const int motorID)
{
    m_PivotMotor.reset(new CowLib::CowMotorController(motorID, ""));
    m_PivotMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    ResetConstants();
}

void Pivot::RequestAngle(double angle)
{
    m_MotorRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("PIVOT_GEAR_RATIO"));
}

double Pivot::GetAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetPosition(), CONSTANT("PIVOT_GEAR_RATIO"));
}

void Pivot::UpdatePID(double armExt)
{
    double p
        = CONSTANT("PIVOT_P_BASE") + (CONSTANT("PIVOT_P_EXTENSION") * armExt) + (CONSTANT("PIVOT_P_ANGLE") * armExt);

    m_PivotMotor->SetPID(p, CONSTANT("PIVOT_I"), CONSTANT("PIVOT_D"), CONSTANT("PIVOT_F"));
}

void Pivot::ResetConstants()
{
    m_PivotMotor->SetPID(CONSTANT("PIVOT_P"), CONSTANT("PIVOT_I"), CONSTANT("PIVOT_D"), CONSTANT("PIVOT_F"));
}

void Pivot::Handle()
{
    m_PivotMotor->Set(m_MotorRequest);
}