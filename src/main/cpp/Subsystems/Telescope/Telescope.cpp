//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Telescope.h"

Telescope::Telescope(const int MotorId)
{
    m_TelescopeMotor.reset(new CowLib::CowMotorController(MotorId));
    // don't think we want the rotator to be in brake mode so we can check max and min
    // m_RotatorMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
}

void Telescope::SetPosition()
{
    m_TelescopeMotor->Set(m_MotorRequest);

    m_Position = m_TelescopeMotor->GetPosition() / CONSTANT("TELESCOPE_RATIO");
}

void Telescope::RequestPosition(double pos)
{
    m_MotorRequest.Position = pos * CONSTANT("TELESCOPE_RATIO");
}

void Telescope::ResetConstants()
{
    m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_P"),
                             CONSTANT("TELESCOPE_I"),
                             CONSTANT("TELESCOPE_D"),
                             CONSTANT("TELESCOPE_F"));
}