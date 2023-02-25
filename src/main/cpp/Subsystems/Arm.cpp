//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Arm.h"

#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm(int pivotMotor, int telescopeMotor, int wristMotor, int intakeMotor, int solenoidChannel)
{
    // Set ArmInterface Members
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXT");
    m_MaxPos   = CONSTANT("ARM_MAX_EXT");

    m_WristMaxAngle = CONSTANT("WRIST_MAX_ANGLE");

    m_FrameHeight = CONSTANT("AFRAME_HEIGHT");
    m_ClawLen     = CONSTANT("CLAW_LEN");

    m_LoopCount = 0;

    m_ArmInvert  = false;
    m_WristState = false;

    m_PivotLockout = false;
    m_ExtLockout   = false;

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    m_Pivot     = std::make_unique<Pivot>(pivotMotor);
    m_Claw      = std::make_unique<Claw>(wristMotor, intakeMotor, solenoidChannel);

    ResetConstants();
}

double Arm::GetSafeAngle(double reqAngle, const double curAngle, const double curExt)
{
    // if arm extended and change in angle is large?, do not move pivot
    if (curExt > m_MinPos * 1.05 && (fabs(curAngle) < fabs(reqAngle) * 0.95 || fabs(curAngle) > fabs(reqAngle) * 1.05))
    {
        m_PivotLockout = true;
        return curAngle;
    }

    m_PivotLockout = false;

    // If Arm Angle is greater >= Max
    if (reqAngle > m_MaxAngle)
    {
        // Set the current angle to MaxAngle
        reqAngle = m_MaxAngle;
    }
    // If Arm Angle is greater <= Min
    else if (reqAngle < m_MinAngle)
    {
        // Set the current angle to MinAngle and position to MaxPosAtMinAngle
        reqAngle = m_MinAngle;
    }

    // lock out extension until we reach desired angle +/- a few degrees
    if (fabs(curAngle) < fabs(reqAngle) * 0.95 || fabs(curAngle) > fabs(reqAngle) * 1.05)
    {
        m_ExtLockout = true;
    }
    else
    {
        m_ExtLockout = false;
    }

    return reqAngle;
}

double Arm::GetSafeExt(double position, const double reqAngle, const double curExt)
{
    // if pivot is locked out or we would pivot inside bot (not sure if this one really works)
    // set bot arm to minimum extension
    if (fabs(reqAngle) > CONSTANT("PIVOT_WITHIN_BOT") || m_PivotLockout)
    {
        return m_MinPos;
    }
    else if (m_ExtLockout) // if we are locked out from extending, return current extension
    {
        return curExt;
    }

    double maxExtAllowed = m_MaxPos;

    // TODO: check this math - also take claw into account
    // if the telescope would potentially point at the ground - check distance to ground
    if (reqAngle > 90 || reqAngle < -90) // potential to point into ground
    {
        double curAngleRads = (fabs(reqAngle) - 90) * M_PI / 180;

        // TODO: should subtract this by claw length depending on orientation of claw?
        maxExtAllowed = std::min(m_FrameHeight / std::sin(curAngleRads), m_MaxPos);
    }

    // check against max and min
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

double Arm::GetSafeWristAngle(double curPivotAngle, double reqPivotAngle)
{
    // determines safe wrist angle from requested pivot angle
    // this code assumes 0 degrees on wrist is directly in line with arm - THIS MAY NOT BE THE CASE IN REALITY
    // generally, this will be horizontal to the floor
    // there are 3 cases where that does not follow however
    //  1. when intaking and the flip wrist button has been pressed for cone pickup
    //       and wrist is perpendicular to the floor (TODO: this should also change safe pivot angle by a few degrees)
    //  2. when inside bot perimeter
    //  3. when arm is between around 20 to -20 degrees (above bot) as not not cause major issues
    //       when flipping arm orientation

    if (curPivotAngle < 20 && curPivotAngle > -20)
    {
        return 0;
    }
    else if (fabs(reqPivotAngle) > CONSTANT("PIVOT_WITHIN_BOT"))
    {
        // theoretically, wrist angle should be opposite to pivot angle?
        return reqPivotAngle > 0 ? m_WristMaxAngle * -1 : m_WristMaxAngle;
    }

    // in the standard case 90 - anglePivot + flippedOffset = angleWrist - assuming we have our +/- correct
    double flipOffset = 0;
    if (m_WristState)
    {
        // add an additional 90 degrees to handle when wrist is vertical
        flipOffset = 90;
    }
    // depending on the angle of the pivot, our math changes slightly
    double angle = reqPivotAngle > 0 ? ((90 + flipOffset) - reqPivotAngle) : (-(90 + flipOffset) - reqPivotAngle);

    // I don't think that it's possible to come up with a number outside the range of the wrist
    // but check just in case
    if (fabs(angle) > m_WristMaxAngle)
    {
        // carries over sign
        angle = (m_WristMaxAngle * angle / fabs(angle));
    }

    return angle;
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
 * @brief swap +/- for arm rotation
*/
void Arm::InvertArm(bool value)
{
    m_ArmInvert = value;
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
    // potentially need to add another if for stow so cargo does not fall out?
    if (m_State == ARM_IN)
    {
        m_Claw->SetIntakeSpeed(1);
    }
    else
    {
        m_Claw->SetIntakeSpeed(0);
    }
}

void Arm::FlipWristState()
{
    m_WristState = !m_WristState;
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
    // reset wrist position when transitioning states
    if (state != ARM_IN && state != ARM_MANUAL)
    {
        m_WristState = false;
    }

    m_State = state;
}

void Arm::ResetConstants()
{
    m_Cargo    = ST_NONE;
    m_State    = ARM_NONE;
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXT");
    m_MaxPos   = CONSTANT("ARM_MAX_EXT");

    m_WristMaxAngle = CONSTANT("WRIST_MAX_ANGLE");

    m_FrameHeight = CONSTANT("AFRAME_HEIGHT");
    m_ClawLen     = CONSTANT("CLAW_LEN");

    m_Pivot->ResetConstants();
    m_Telescope->ResetConstants();
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

    if (m_WristState && m_State == ARM_IN)
    {
        // modify setpoints slightly TODO: figure these out
        angle     = angle > 0 ? angle - 3 : angle + 3;
        extension = extension + 3;
    }
    if (m_ArmInvert)
    {
        angle = angle * -1;
    }

    double safeAngle = GetSafeAngle(angle, curAngle, curExt);
    m_Pivot->RequestAngle(safeAngle);
    double safeExt = GetSafeExt(extension, safeAngle, curExt);
    m_Telescope->RequestPosition(safeExt);
    double safeWrist = GetSafeWristAngle(curAngle, safeAngle);
    m_Claw->RequestWristAngle(safeWrist);

    double wristAngle = m_Claw->GetWristAngle();

    static char *translateArr[8] = { "none", "in", "stow", "L3", "L2", "L1", "score", "manual" };
    frc::SmartDashboard::PutString("arm/state", translateArr[m_State]);
    frc::SmartDashboard::PutNumber("arm/pivot safe angle", safeAngle);
    frc::SmartDashboard::PutNumber("arm/telescope safe ext", safeExt);
    frc::SmartDashboard::PutNumber("arm/wrist safe angle", safeWrist);
    frc::SmartDashboard::PutNumber("arm/pivot orig angle", angle);
    frc::SmartDashboard::PutNumber("arm/telescope orig ext", extension);
    frc::SmartDashboard::PutNumber("arm/telescope ext", curExt);
    frc::SmartDashboard::PutNumber("arm/pivot angle", curAngle);
    frc::SmartDashboard::PutNumber("arm/wrist angle", wristAngle);
}

void Arm::Handle()
{
    // PID update check
    if (m_LoopCount++ % 10 == 0) // fires every 200ms
    {
        double telescopePos = m_Telescope->GetPosition();
        // m_Pivot->UpdatePID(telescopePos);
        // m_Telescope->UpdatePID(telescopePos);
        m_LoopCount = 1;
    }

    m_Pivot->Handle();
    m_Telescope->Handle();
    m_Claw->Handle();
}