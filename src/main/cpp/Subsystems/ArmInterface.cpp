//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "ArmInterface.h"

#include <cmath>
#include <sstream>
#include <stdexcept>

void ArmInterface::SetAngle(double angle)
{
    SetSafeAngle(angle);
    SetArmAngle(m_CurrentConfig.angle);
    SetArmExtension(m_CurrentConfig.ext);
}

void ArmInterface::SetTelescopePosition(double position)
{
    SetSafePos(position);
    SetArmExtension(m_CurrentConfig.ext);
}

void ArmInterface::SetSafeAngle(const double angle)
{
    // If Arm Angle is greater >= Max
    if (angle >= m_MaxAngle)
    {
        // Set the current angle to MaxAngle
        m_CurrentConfig.angle = m_MaxAngle;
    }
    // If Arm Angle is greater <= Max
    else if (angle <= m_MinAngle)
    {
        // Set the current angle to MinAngle and position to MaxPosAtMinAngle
        m_CurrentConfig.angle = m_MinAngle;
    }
    else
    {
        m_CurrentConfig.angle = angle;
    }

    // Check if the currenet Pos is safe at the new angle
    SetSafePos(m_CurrentConfig.ext);
}

void ArmInterface::SetSafePos(const double position)
{
    // Throw is angle is invalid as this could damage bot!
    if (m_CurrentConfig.angle > m_MaxAngle || m_CurrentConfig.angle < m_MinAngle)
    {
        std::stringstream ss;
        ss << "Error: Angle is Invalid: " << m_CurrentConfig.angle << "!";
        throw std::runtime_error(ss.str());
    }
    const double totalHeight          = m_FrameHeight + m_ClawLen;
    const double MaxPosAtCurrentAngle = totalHeight / (std::cos(m_CurrentConfig.angle * M_PI / 180.0));
    if (position > MaxPosAtCurrentAngle)
    {
        m_CurrentConfig.ext = MaxPosAtCurrentAngle;
    }
    else
    {
        m_CurrentConfig.ext = position;
    }
}
