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
    m_MaxPos   = CONSTANT("ARM_MAX_EXT") * CONSTANT("TELESCOPE_GEAR_RATIO");

    m_WristMaxAngle = CONSTANT("WRIST_MAX_ANGLE");

    m_FrameHeight = CONSTANT("AFRAME_HEIGHT");
    m_ClawLen     = CONSTANT("CLAW_LEN");

    m_LoopCount = 0;

    // this should match our starting configuration
    m_ArmInvert  = true;
    m_WristState = false;

    m_PivotLockout = false;
    m_ExtLockout   = false;

    m_Telescope = std::make_unique<Telescope>(telescopeMotor);
    m_Pivot     = std::make_unique<Pivot>(pivotMotor);
    m_Claw      = std::make_unique<Claw>(wristMotor, intakeMotor, solenoidChannel);

    m_PrevState      = ARM_NONE;
    m_Cargo          = CG_CONE;
    m_ResetCargoFlag = false;

    m_ManualControl = false;

    m_UpdateArmLPF = false;
    m_ReInitArmLPF = false;
    m_ArmLPF       = new CowLib::CowLPF(CONSTANT("ARM_LPF_BETA"));

    m_ClawState = CLAW_OFF;

    m_BrakeMode = true;

    ResetConstants();
}

double Arm::GetSafeAngle(double reqAngle, const double curAngle, const double curExt)
{
    // if arm extended and change in angle is large?, do not move pivot
    if (curExt > m_MinPos + 0.5
        && (fabs(curAngle) < fabs(reqAngle) - CONSTANT("PIVOT_WHILE_EXT_TOLERANCE")
            || fabs(curAngle) > fabs(reqAngle) + CONSTANT("PIVOT_WHILE_EXT_TOLERANCE")
            || std::signbit(reqAngle) != std::signbit(curAngle)))
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
    if (fabs(curAngle) < fabs(reqAngle) - CONSTANT("PIVOT_WHILE_EXT_TOLERANCE")
        || fabs(curAngle) > fabs(reqAngle) + CONSTANT("PIVOT_WHILE_EXT_TOLERANCE")
        || std::signbit(curAngle) != std::signbit(reqAngle))
    {
        m_ExtLockout = true;
    }
    else
    {
        m_ExtLockout   = false;
        m_UpdateArmLPF = false;
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

    // TODO: redo or check this math - ext is based on encoder units and not inches
    // if the telescope would potentially point at the ground - check distance to ground
    // if (reqAngle > 90 || reqAngle < -90) // potential to point into ground
    // {
    //     double curAngleRads = (fabs(reqAngle) - 90) * M_PI / 180;

    //     // TODO: should subtract this by claw length depending on orientation of claw?
    //     maxExtAllowed = std::min(m_FrameHeight / std::sin(curAngleRads), m_MaxPos);
    // }

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
    // this code assumes 0 degrees on wrist is at a right angle to the arm
    // generally, this will be horizontal to the floor
    // there are 3 cases where that does not follow however
    //  1. when intaking and the flip wrist button has been pressed for cone pickup
    //       and wrist is perpendicular to the floor (TODO: this should also change safe pivot angle by a few degrees)
    //  2. when inside bot perimeter
    // if (reqPivotAngle < 20 && reqPivotAngle > -20 && m_State != ARM_HUMAN) // keep in temporarily
    // {
    //     return 90;
    // }
    if (fabs(reqPivotAngle) >= CONSTANT("PIVOT_WITHIN_BOT") || m_State == ARM_STOW)
    {
        // theoretically, wrist angle should be opposite to pivot angle?
        return reqPivotAngle > 0 ? m_WristMaxAngle : 0;
    }
    // else if (m_State == ARM_GND || m_State == ARM_L2 || m_State == ARM_L3)
    // {
    //     // TODO: make these constants
    //     return reqPivotAngle > 0 ? m_WristMaxAngle - 50 : 50;
    // }

    // in the standard case 90 - anglePivot + flippedOffset = angleWrist - assuming we have our +/- correct
    double flipOffset = 0;
    if (m_WristState)
    {
        // add an additional 90 degrees to handle when wrist is vertical
        flipOffset = CONSTANT("WRIST_FLIP_OFFSET");
    }
    // depending on the angle of the pivot, our math changes slightly
    // double angle = reqPivotAngle > 0 ? (180 - reqPivotAngle + flipOffset) : (fabs(reqPivotAngle) - flipOffset);
    double angle = reqPivotAngle > 0 ? reqPivotAngle - flipOffset : (180 - fabs(reqPivotAngle) + flipOffset);

    // I don't think that it's possible to come up with a number outside the range of the wrist
    // but check just in case
    if (angle > m_WristMaxAngle)
    {
        angle = m_WristMaxAngle;
    }
    else if (angle < 0)
    {
        angle = 0;
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

CLAW_STATE Arm::GetClawState()
{
    return m_ClawState;
}

/**
 * @brief swap +/- for arm rotation
*/
void Arm::InvertArm(bool value)
{
    if (value != m_ArmInvert)
    {
        m_UpdateArmLPF = true;
    }
    m_ArmInvert = value;
}

/**
 * @brief state controller for Claw/Intake
 * should be relatively simple logic control based on current state
 * CG_NONE for cargo should not change solenoid
 */
void Arm::UpdateClawState()
{
    bool open = false;

    // set cargo open/close state
    if (m_Cargo == CG_CUBE)
    {
        open = true;
    }
    else if (m_Cargo == CG_CONE)
    {
        open = false;
    }

    switch (m_ClawState)
    {
    case CLAW_OFF :
        if (m_Cargo == CG_CONE)
        {
            m_Claw->SetIntakeSpeed(CONSTANT("CLAW_OFF_CONE"));
        }
        else
        {
            m_Claw->SetIntakeSpeed(CONSTANT("CLAW_OFF_CUBE"));
        }

        if (m_ResetCargoFlag)
        {
            // not sure if setting to NONE or CUBE is better here
            // but this is how you get around the top check
            // alternatively, can keep this flag set and change to
            // false in intake without changing cargo
            // m_Cargo          = CG_NONE;
            open = true;
            // m_ResetCargoFlag = false;
        }
        break;
    case CLAW_INTAKE :
        m_ResetCargoFlag = false;
        if (m_Cargo == CG_CUBE)
        {
            m_Claw->SetIntakeSpeed(CONSTANT("CLAW_INTAKE_CUBE"));
        }
        else if (m_Cargo == CG_CONE)
        {
            m_Claw->SetIntakeSpeed(CONSTANT("CLAW_INTAKE_CONE"));
        }
        break;
    case CLAW_EXHAUST :
        if (m_Cargo == CG_CUBE)
        {
            if (m_State == ARM_GND)
            {
                m_Claw->SetIntakeSpeed(CONSTANT("CLAW_SCORE_CUBE_GND"));
            }
            else
            {
                m_Claw->SetIntakeSpeed(CONSTANT("CLAW_SCORE_CUBE"));
            }
        }
        else if (m_Cargo == CG_CONE)
        {
            if (m_State == ARM_STOW)
            {
                m_Claw->SetIntakeSpeed(CONSTANT("CLAW_STOW_CONE"));
            }
            else
            {
                m_Claw->SetIntakeSpeed(CONSTANT("CLAW_L3_SCORE_CONE"));
            }
            // if (m_State == ARM_L3)
            // {
            // }
            // else
            // {
            //     m_Claw->SetIntakeSpeed(CONSTANT("CLAW_OFF_SPEED"));
            // }
            open             = true;
            m_ResetCargoFlag = true;
        }
        break;
    case CLAW_NONE :
        break;
    }

    m_Claw->SetOpen(open);
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
    if (m_ClawState == CLAW_INTAKE)
        m_Cargo = cargo;
}

void Arm::SetArmState(ARM_STATE state)
{
    // reset wrist position when transitioning states
    if (m_ClawState != CLAW_INTAKE && !m_ManualControl && state != ARM_GND)
    {
        m_WristState = false;
    }

    if (m_State != state)
    {
        m_ReInitArmLPF = true;
    }

    m_State = state;
}

void Arm::SetClawState(CLAW_STATE state)
{
    m_ClawState = state;
}

void Arm::ResetConstants()
{
    m_Cargo    = CG_NONE;
    m_State    = ARM_NONE;
    m_MaxAngle = CONSTANT("PIVOT_MAX_ANGLE");
    m_MinAngle = CONSTANT("PIVOT_MAX_ANGLE") * -1;
    m_MinPos   = CONSTANT("ARM_MIN_EXT");
    m_MaxPos   = CONSTANT("ARM_MAX_EXT") * CONSTANT("TELESCOPE_GEAR_RATIO");

    m_WristMaxAngle = CONSTANT("WRIST_MAX_ANGLE");

    m_FrameHeight = CONSTANT("AFRAME_HEIGHT");
    m_ClawLen     = CONSTANT("CLAW_LEN");

    m_Pivot->ResetConstants();
    m_Telescope->ResetConstants();
    m_Claw->ResetConstants();

    m_ArmLPF->ReInit(m_Pivot->GetSetpoint(), m_Pivot->GetSetpoint());
}

void Arm::CheckMinMax()
{
    return;
}

bool Arm::AtTarget()
{
    return m_Pivot->AtTarget() && m_Telescope->AtTarget() && m_Claw->WristAtTarget();
}

void Arm::RequestPosition(double angle, double extension, double clawOffset)
{
    if (m_ManualControl)
    {
        return;
    }

    double curAngle = m_Pivot->GetAngle();
    double curExt   = m_Telescope->GetPosition();

    if (m_WristState && m_State == ARM_GND)
    {
        // modify setpoints slightly when wrist is vertical TODO: figure these out
        angle = angle > 0 ? angle - CONSTANT("PIVOT_WHEN_VERT") : angle + CONSTANT("PIVOT_WHEN_VERT");
        // extension = extension + 3;
    }
    if (m_ArmInvert)
    {
        angle = angle * -1;
    }

    double safeAngle = GetSafeAngle(angle, curAngle, curExt);

    if (m_ReInitArmLPF)
    {
        // this is set true by SetArmState() when state changes
        m_ArmLPF->ReInit(curAngle, curAngle);
        m_ReInitArmLPF = false;
    }

    if (m_UpdateArmLPF)
    {
        // this is set to true by InvertArm() when state changes
        // this is set to false by GetSafeAngle() when arm is within 5 degrees of target
        safeAngle = m_ArmLPF->Calculate(safeAngle);
    }

    m_Pivot->RequestAngle(safeAngle);

    double safeExt = GetSafeExt(extension, safeAngle, curExt);

    // if (fabs(curExt - safeExt) < CONSTANT("TELESCOPE_PID_SWAP_THRESHOLD"))
    // {
    //     if (curExt >= safeExt)
    //     {
    //         // Use down constants
    //         m_Telescope->UsePIDSet(Telescope::RETRACTING);
    //     }
    //     else
    //     {
    //         // Use up constants
    //         m_Telescope->UsePIDSet(Telescope::EXTENDING);
    //     }
    // }
    m_Telescope->RequestPosition(safeExt);

    double safeWrist = GetSafeWristAngle(curAngle, safeAngle);

    if (!m_WristState)
    {
        safeWrist = angle > 0 ? safeWrist - clawOffset : safeWrist + clawOffset;
    }

    m_Claw->RequestWristAngle(safeWrist);

    // if (m_LoopCount % 20 == 0) // fires every 400ms
    // {
    //     CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "angle: %f\text: %f\n", safeAngle, safeExt);
    //     CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "wrist: %f\n", safeWrist);
    // }

    m_PrevState = GetArmState();
}

void Arm::ManualPosition(double value, bool pivotOrTelescope)
{
    double curAngle = m_Pivot->GetAngle();
    double curExt   = m_Telescope->GetPosition();

    if (pivotOrTelescope) // pivot = true
    {
        if (m_ArmInvert)
        {
            value = value * -1;
        }
        if (curAngle > 0)
        {
            if (value < 0)
            {
                curAngle += value * CONSTANT("PIVOT_MANUAL_CTRL") * 3;
            }
            else
            {
                curAngle += value * CONSTANT("PIVOT_MANUAL_CTRL");
            }
        }
        else
        {
            if (value > 0)
            {
                curAngle += value * CONSTANT("PIVOT_MANUAL_CTRL") * 3;
            }
            else
            {
                curAngle += value * CONSTANT("PIVOT_MANUAL_CTRL");
            }
        }

        if (fabs(curAngle) > m_MaxAngle)
        {
            curAngle = m_MaxAngle * curAngle / fabs(curAngle);
        }
        m_Pivot->RequestAngle(curAngle);
    }
    else
    {
        curExt += value * CONSTANT("EXT_MANUAL_CTRL");
        if (curExt > m_MaxPos)
        {
            curExt = m_MaxPos;
        }
        if (curExt < m_MinPos)
        {
            curExt = m_MinPos;
        }
        m_Telescope->RequestPosition(curExt);
    }

    double safeWrist = GetSafeWristAngle(curAngle, curAngle);
    if (m_LoopCount % 20 == 0) // fires every 400ms
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "angle: %f\text: %f\n", curAngle, curExt);
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "manual wrist: %f\n", safeWrist);
    }
}

void Arm::RequestSafeStow()
{
    double curAngle = m_Pivot->GetAngle();
    double reqAngle = CONSTANT("SCORE_STOW_ANGLE");
    // reqAngle = curAngle;

    if (m_ArmInvert)
    {
        reqAngle = reqAngle * -1;
    }

    if (m_ReInitArmLPF)
    {
        // this is set true by SetArmState() when state changes
        m_ArmLPF->ReInit(curAngle, curAngle);
        m_ReInitArmLPF = false;
    }

    if (fabs(curAngle) > fabs(reqAngle) - 5 && fabs(curAngle) < fabs(reqAngle) + 5
        && std::signbit(curAngle) == std::signbit(reqAngle))
    {
        // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "at target\ncur: %f\nreq: %f\n", curAngle, reqAngle);
        m_UpdateArmLPF = false;
    }

    if (m_UpdateArmLPF)
    {
        // this is set to true by InvertArm() when state changes
        // this is set to false by GetSafeAngle() when arm is within 5 degrees of target
        reqAngle = m_ArmLPF->Calculate(reqAngle);
    }

    m_Pivot->RequestAngle(reqAngle);

    if (fabs(curAngle) > fabs(reqAngle) - CONSTANT("DRIVER_STOW_THRESHOLD")
        && fabs(curAngle) < fabs(reqAngle) + CONSTANT("DRIVER_STOW_THRESHOLD"))
    {
        m_Telescope->RequestPosition(CONSTANT("SCORE_STOW_EXT"));
    }

    double wristAngle = m_Pivot->GetAngle() > 0 ? 135 : 45;
    m_Claw->RequestWristAngle(wristAngle);
}

void Arm::Handle()
{
    // PID update check
    if (m_LoopCount++ % 10 == 0) // fires every 200ms
    {
        // double telescopePos = m_Telescope->GetPosition();
        // m_Pivot->UpdatePID(telescopePos);
        // m_Telescope->UpdatePID(telescopePos);
        m_LoopCount = 1;
    }

    m_Pivot->Handle();
    m_Telescope->Handle();
    m_Claw->Handle();
}

void Arm::UseManualControl(bool manual)
{
    m_ManualControl = manual;
}

void Arm::SetBrakeMode(bool brakeMode)
{
    if (m_BrakeMode == brakeMode)
    {
        return;
    }

    m_BrakeMode = brakeMode;

    m_Pivot->BrakeMode(brakeMode);
    m_Telescope->BrakeMode(brakeMode);
    m_Claw->BrakeMode(brakeMode);
}
