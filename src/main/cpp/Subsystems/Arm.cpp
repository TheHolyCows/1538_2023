//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// Climber.h
// author: Goober Gang
// created on: 2023-1-14
//==================================================

#include "Arm.h"

Arm::Arm(int leftMotor, int rightMotor)
{
    m_LeftMotor  = new CowLib::CowMotorController(leftMotor);
    m_RightMotor = new CowLib::CowMotorController(rightMotor);

    m_LeftMotor->SetControlMode(CowLib::CowMotorController::POSITION);
    m_RightMotor->SetControlMode(CowLib::CowMotorController::POSITION);

    m_LeftMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    m_RightMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);
    //set one motor to inverted
    m_RightMotor->GetInternalMotor()->SetSensorPhase(true);
    m_RightMotor->GetInternalMotor()->SetSensorPhase(false);
    

    m_Position  = 0;

    ResetConstants();
}

void Arm::SetPosition(double position)
{
    m_Position = position;
}


double Arm::GetPosition()
{
    return m_RightMotor->GetPosition();
}



void Arm::ResetConstants()
{
    m_LeftMotor->SetPIDGains(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), 0, 1);
    m_RightMotor->SetPIDGains(CONSTANT("ARM_P"), CONSTANT("ARM_I"), CONSTANT("ARM_D"), 0, 1);
}

void Arm::handle()
{
    //one of these must be inverted
    if (m_LeftMotor)
    {
        m_LeftMotor->Set(m_Position);
    }

    if (m_RightMotor)
    {
        m_RightMotor->Set(m_Position);
    }
}

Arm::~Arm()
{
    delete m_LeftMotor;
    delete m_RightMotor;
}