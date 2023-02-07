//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#pragma once

#include "Pivot.h"

Pivot::Pivot(const int MotorId)
{
    m_PivotMotor.reset(new CowLib::CowMotorController(MotorId));
}

void Pivot::SetAngle()
{
    if (m_LoopCount++ % 20 == 0)
    {
        double p
            = CONSTANT("ARM_P_BASE") + (CONSTANT("ARM_P_EXTENSION") * m_Angle) + (CONSTANT("ARM_P_ANGLE") * m_Angle);

        m_PivotMotor->SetPID(p, CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
    }

    m_PivotMotor->Set(m_MotorRequest);

    m_Angle = CowLib::Conversions::FalconToDegrees(m_PivotMotor->GetPosition(), CONSTANT("ARM_ROTATION_GEAR_RATIO"));
}

void Pivot::RequestAngle(double angle)
{
    m_MotorRequest.Position = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("ARM_ROTATION_GEAR_RATIO"));
}

void Pivot::ResetConstants()
{
    m_PivotMotor->SetPID(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
}