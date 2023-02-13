//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Arm.h"

#include "ArmMock.h"

Arm::Arm(int rotationMotor, int telescopeMotor)
{
    // Set ArmInterface Members
    m_MinAngle = 0;
    m_MaxAngle = 0;
    m_MinPos   = 0;
    m_MaxPos   = 0;

    m_RotationMotor.reset(new CowLib::CowMotorController(rotationMotor));
    m_TelescopeMotor.reset(new CowLib::CowMotorController(telescopeMotor));
    m_RotationControlRequest  = { 0 };
    m_TelescopeControlRequest = { 0 };
    m_LoopCount               = 0;
    // no longer works with Phoenix Pro
    // m_RotatorMotor->SetControlMode(CowLib::CowMotorController::POSITION);
    // m_TelescopeMotor->SetControlMode(CowLib::CowMotorController::POSITION);

    // don't think we want the rotator to be in brake mode so we can check max and min
    // m_RotatorMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    ResetConstants();
}

void Arm::SetArmAngle(double angle)
{
    m_RotationControlRequest.Position
        = CowLib::Conversions::DegreesToFalcon(angle, CONSTANT("ARM_ROTATION_GEAR_RATIO"));
}

void Arm::SetArmPosition(double position)
{
    // TODO: check this math
    m_TelescopeControlRequest.Position = position * CONSTANT("TELESCOPE_RATIO");
}

void Arm::ResetConstants()
{
    m_RotationMotor->SetPID(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
    m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_P"),
                             CONSTANT("TELESCOPE_I"),
                             CONSTANT("TELESCOPE_D"),
                             CONSTANT("TELESCOPE_F"));
}

void Arm::Handle()
{
    if (m_LoopCount++ % 20 == 0)
    {
        double p = CONSTANT("ARM_P_BASE") + (CONSTANT("ARM_P_EXTENSION") * m_CurrentConfig.pos)
                   + (CONSTANT("ARM_P_ANGLE") * m_CurrentConfig.angle);

        m_RotationMotor->SetPID(p, CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
    }

    if (m_RotationMotor)
    {
        m_RotationMotor->Set(m_RotationControlRequest);

        m_CurrentConfig.angle
            = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("ARM_ROTATION_GEAR_RATIO"));
    }

    if (m_TelescopeMotor)
    {
        m_TelescopeMotor->Set(m_TelescopeControlRequest);

        m_CurrentConfig.pos = m_TelescopeMotor->GetPosition() / CONSTANT("TELESCOPE_RATIO");
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
    // Find the angle that is arm straight up
    double zeroAngle = m_MinAngle + ((m_MaxAngle - m_MinAngle) / 2.0);

    // Want to ensure current angle is up to date
    m_CurrentConfig.angle
        = CowLib::Conversions::FalconToDegrees(m_RotationMotor->GetPosition(), CONSTANT("ARM_ROTATION_GEAR_RATIO"));

    // Set to current angle - straight up angle, changing scope
    m_RotationMotor->SetSensorPosition(
        CowLib::Conversions::DegreesToFalcon(m_CurrentConfig.angle - zeroAngle, CONSTANT("ARM_ROTATION_GEAR_RATIO")));
}