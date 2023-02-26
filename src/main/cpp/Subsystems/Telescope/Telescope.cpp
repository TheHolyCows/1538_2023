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
    m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    ResetConstants();
}

void Telescope::RequestPosition(double pos)
{
    m_MotorRequest.Position = pos;
}

double Telescope::GetPosition()
{
    return m_TelescopeMotor->GetPosition();
}

void Telescope::UpdatePID(double armExt)
{
    // todo
}

void Telescope::ResetConstants()
{
    m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_P"),
                             CONSTANT("TELESCOPE_I"),
                             CONSTANT("TELESCOPE_D"),
                             CONSTANT("TELESCOPE_F"));
}

void Telescope::Handle()
{
    m_TelescopeMotor->Set(m_MotorRequest);
}