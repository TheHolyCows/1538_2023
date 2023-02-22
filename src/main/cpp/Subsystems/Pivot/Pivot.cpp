//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Pivot.h"

Pivot::Pivot(const int motorID)
{
    m_PivotMotor.reset(new CowLib::CowMotorController(motorID));

    ResetConstants();
}

void Pivot::RequestAngle(double angle)
{
    m_MotorRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("ARM_ROTATION_GEAR_RATIO"));
}

double Pivot::GetAngle()
{
    return CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetPosition(), CONSTANT("ARM_ROTATION_GEAR_RATIO"));
}

void Pivot::UpdatePID(double armExt)
{
    double p = CONSTANT("ARM_P_BASE") + (CONSTANT("ARM_P_EXTENSION") * armExt) + (CONSTANT("ARM_P_ANGLE") * armExt);

    m_PivotMotor->SetPID(p, CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
}

void Pivot::ResetConstants()
{
    m_PivotMotor->SetPID(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
}

void Pivot::Handle()
{
    m_PivotMotor->Set(m_MotorRequest);
}