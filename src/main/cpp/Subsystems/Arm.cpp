//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Arm.h"

Arm::Arm(int rotatorMotor, int telescopeMotor)
{
    m_RotatorMotor   = new CowLib::CowMotorController(rotatorMotor);
    m_TelescopeMotor = new CowLib::CowMotorController(telescopeMotor);

    // no longer works with Phoenix Pro
    // m_RotatorMotor->SetControlMode(CowLib::CowMotorController::POSITION);
    // m_TelescopeMotor->SetControlMode(CowLib::CowMotorController::POSITION);

    // don't think we want the rotator to be in brake mode so we can check max and min
    // m_RotatorMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    // check these
    // m_RotatorMotor->SetInverted(true);
    // m_TelescopeMotor->SetInverted(false);

    m_RotatorController.Position   = 0;
    m_TelescopeController.Position = 0;

    ResetConstants();
}

void Arm::SetRotatorPos(double position)
{
    m_RotatorController.Position = position;
}

void Arm::SetTelescopePos(double position)
{
    m_TelescopeController.Position = position;
}

double Arm::GetRotatorPos()
{
    return m_RotatorMotor->GetPosition();
}

double Arm::GetTelescopePos()
{
    return m_TelescopeMotor->GetPosition();
}

void Arm::ResetConstants()
{
    m_RotatorMotor->SetPID(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
    m_TelescopeMotor->SetPID(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), CONSTANT("ARM_F"));
}

void Arm::Handle()
{
    if (m_RotatorMotor)
    {
        m_RotatorMotor->Set(m_RotatorController);
    }

    if (m_TelescopeMotor)
    {
        m_TelescopeMotor->Set(m_TelescopeController);
    }
}

Arm::~Arm()
{
    delete m_RotatorMotor;
    delete m_TelescopeMotor;
}