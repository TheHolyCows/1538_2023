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
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXTENSION");
    m_MaxPos   = CONSTANT("ARM_MAX_EXTENSION");

    m_LoopCount = 0;

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    // m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    m_Pivot = std::make_unique<Pivot>(pivotMotor);
    m_Claw  = std::make_unique<Claw>(wristMotor, intakeMotor, solenoidChannel);

    ResetConstants();
}

double Arm::GetSafeAngle(double angle)
{
    // If Arm Angle is greater >= Max
    if (angle >= m_MaxAngle)
    {
        // Set the current angle to MaxAngle
        angle = m_MaxAngle;
    }
    // If Arm Angle is greater <= Max
    else if (angle <= m_MinAngle)
    {
        // Set the current angle to MinAngle and position to MaxPosAtMinAngle
        angle = m_MinAngle;
    }

    return angle;
}

double Arm::GetSafeExt(double position)
{
    double curAngle             = m_Pivot->GetAngle();
    double totalHeight          = m_ArmHeight + m_ClawHeight;
    double MaxPosAtCurrentAngle = totalHeight / (std::cos(m_CurrentConfig.angle * M_PI / 180.0));

    if (position > MaxPosAtCurrentAngle)
    {
        m_CurrentConfig.ext = MaxPosAtCurrentAngle;
    }

    return position;
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
 * @brief state controller for Claw/Intake
 * should be relatively simple logic control based on current state
 * ST_NONE for cargo should not change solenoid
 */
void Arm::UpdateClawState()
{
    // set cargo open/close state
    if (m_Cargo == ST_CUBE)
    {
        m_Claw->SetOpen(true);
    }
    else if (m_Cargo == ST_CONE)
    {
        m_Claw->SetOpen(false);
    }

    // set intake on off state - will add exfil state for scoring in future
    if (m_State == ARM_IN)
    {
        m_Claw->SetIntakeSpeed(1);
    }
    else
    {
        m_Claw->SetIntakeSpeed(0);
    }
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
    m_Cargo    = ST_NONE;
    m_State    = ARM_NONE;
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXTENSION");
    m_MaxPos   = CONSTANT("ARM_MAX_EXTENSION");
    m_Pivot->ResetConstants();
    // m_Telescope->ResetConstants();
    m_Claw->ResetConstants();
}

void Arm::CheckMinMax()
{
    return;
}

void Arm::RequestPosition(double angle, double extension)
{
    m_Pivot->RequestAngle(GetSafeAngle(angle));
    GetSafeExt(extension);
}

void Arm::Handle()
{
    // PID update check
    if (m_LoopCount++ % 10 == 0) // fires every 200ms
    {
        m_Pivot->UpdatePID(m_Telescope->GetPosition());
        // m_Telescope->UpdatePID();
        m_LoopCount = 1;
    }

    m_Pivot->Handle();
    // m_Telescope->Handle();
    m_Claw->Handle();
}