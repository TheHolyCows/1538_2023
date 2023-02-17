//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Arm.h"

Arm::Arm(int pivotMotor, int telescopeMotor, int wristMotor, int intakeMotor, int solenoidChannel)
{
    // Set ArmInterface Members
    m_MinAngle = 0;
    m_MaxAngle = 0;
    m_MinPos   = 0;
    m_MaxPos   = 0;

    // move to pivot
    m_RotationMotor.reset(new CowLib::CowMotorController(pivotMotor));
    m_RotationControlRequest = { 0 };

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    // m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    // m_Pivot     = std::make_unique<Pivot>(pivotMotor);
    m_Claw = std::make_unique<Claw>(wristMotor, intakeMotor, solenoidChannel);

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

/**
 * @brief return current cargo held
*/
ARM_CARGO Arm::GetArmCargo()
{
    return m_Cargo;
}

/**
 * @brief return current state
*/
ARM_STATE Arm::GetArmState()
{
    return m_State;
}

/**
 * @brief sets current cargo of the arm
 * PLEASE CALL AFTER YOU SET STATE
*/
void Arm::SetArmCargo(ARM_CARGO cargo)
{
    if (m_State == ARM_NONE || m_State == ARM_IN)
        m_Cargo = cargo;
}

void Arm::SetArmState(ARM_STATE state)
{
    // don't move arm to in position while scoring?

    if (state == ARM_MANUAL && m_State != ARM_MANUAL)
    {
        // manual control, may change control modes
    }

    m_State = state;
}

void Arm::ResetConstants()
{
    m_Cargo = ST_NONE;
    m_State = ARM_NONE;
    m_Pivot->ResetConstants();
    m_Telescope->ResetConstants();
    m_Claw->ResetConstants();
}

void Arm::Handle()
{
    // m_Pivot->Handle();
    // m_Telescope->Handle();
    m_Claw->Handle();
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