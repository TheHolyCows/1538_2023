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
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXTENSION");
    m_MaxPos   = CONSTANT("ARM_MAX_EXTENSION");

    m_LoopCount = 0;

    m_PivotLockout = false;
    m_ExtLockout   = false;

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    // m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    m_Pivot = std::make_unique<Pivot>(pivotMotor);
    m_Claw  = std::make_unique<Claw>(wristMotor, intakeMotor, solenoidChannel);

    ResetConstants();
}

double Arm::GetSafeAngle(double angle, const double curAngle, const double curExt)
{
    // if arm extended and change in angle is large, do not move pivot
    // this should not be the same as ARM_MIN_EXT? but both should allow us to move within bot perimeter
    if (curExt > CONSTANT("SAFE_EXT_FOR_PIVOT") * 1.05)
    {
        m_PivotLockout = true;
        return curAngle;
    }

    m_PivotLockout = false;

    // If Arm Angle is greater >= Max
    if (angle >= m_MaxAngle)
    {
        // Set the current angle to MaxAngle
        angle = m_MaxAngle;
    }
    // If Arm Angle is greater <= Min
    else if (angle <= m_MinAngle)
    {
        // Set the current angle to MinAngle and position to MaxPosAtMinAngle
        angle = m_MinAngle;
    }

    if (curAngle > angle * 0.95 && curAngle < angle * 1.05)
    {
        m_ExtLockout = false;
    }
    else
    {
        m_ExtLockout = true;
    }

    return angle;
}

double Arm::GetSafeExt(double position, const double curAngle, const double curExt)
{
    if (fabs(curAngle) > CONSTANT("PIVOT_WITHIN_BOT"))
    {
        return m_MinPos;
    }
    else if (m_PivotLockout)
    {
        return CONSTANT("SAFE_EXT_FOR_PIVOT");
    }
    else if (m_ExtLockout)
    {
        return curExt;
    }

    double maxExtAllowed = m_MaxPos;

    // TODO: check this math - also take claw into account
    if (curAngle > 90) // potential to point into ground
    {
        double curAngleRads = (curAngle - 90) * M_PI / 180;
        maxExtAllowed       = std::min(CONSTANT("AFRAME_HEIGHT") / std::sin(curAngleRads), m_MaxPos);
    }

    if (position > maxExtAllowed)
    {
        position = maxExtAllowed;
    }

    if (position < m_MinPos)
    {
        position = m_MinPos;
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
    double curAngle = m_Pivot->GetAngle();
    double curExt   = m_Telescope->GetPosition();

    m_Pivot->RequestAngle(GetSafeAngle(angle, curAngle, curExt));
    m_Telescope->RequestPosition(GetSafeExt(extension, curAngle, curExt));
    //m_Claw->RequestWristAngle(GetSafeWristAngle());
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