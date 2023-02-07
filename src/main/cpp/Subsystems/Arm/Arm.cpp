//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Arm.h"

#include "../Pivot/Pivot.h"
#include "../Telescope/Telescope.h"

Arm::Arm(int rotationMotor, int telescopeMotor)
{
    // Set ArmInterface Members
    m_MinAngle = 0;
    m_MaxAngle = 0;
    m_MinPos   = 0;
    m_MaxPos   = 0;

    m_RotationMotor.reset(new CowLib::CowMotorController(rotationMotor));
    m_RotationControlRequest = { 0 };

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    // Allocate a Telescope Object
    m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    m_Pivot     = std::make_unique<Pivot>(rotationMotor);

    ResetConstants();
}

void Arm::SetArmAngle(double angle)
{
    m_Pivot->RequestAngle(angle);
}

void Arm::SetArmPosition(double position)
{
    // TODO: check this math
    m_Telescope->RequestPosition(position);
}

void Arm::ResetConstants()
{
    m_Pivot->ResetConstants();
    m_Telescope->ResetConstants();
}

void Arm::Handle()
{
    if (m_Pivot)
    {
        m_Pivot->SetAngle();
    }
    if (m_Telescope)
    {
        m_Telescope->SetPosition();
    }
}

void Arm::CheckMinMax()
{
    m_CurrentConfig.angle
        = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("ARM_ROTATION_GEAR_RATIO"));

    if (m_CurrentConfig.angle < m_MinAngle)
    {
        m_MinAngle = m_CurrentConfig.angle;
    }

    if (m_CurrentConfig.angle > m_MaxAngle)
    {
        m_MaxAngle = m_CurrentConfig.angle;
    }
}

void Arm::ZeroSensors()
{
    double angle = m_MinAngle + ((m_MaxAngle - m_MinAngle) / 2.0);

    m_RotationMotor->SetSensorPosition(
        CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("ARM_ROTATION_GEAR_RATIO")));
}