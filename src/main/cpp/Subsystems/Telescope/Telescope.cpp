//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// Arm.h
// author: Cole/JT/Constantine
// created on: 2023-1-14
//==================================================

#include "Telescope.h"

#include <frc/trajectory/TrapezoidProfile.h>
#include <units/velocity.h>

Telescope::Telescope(const int MotorId)
{
    m_TelescopeMotor.reset(new CowLib::CowMotorController(MotorId));
    m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::BRAKE);

    ResetConstants();
}

void Telescope::RequestPosition(double pos)
{
    //    constexpr int flip = -1;

    //    // This is not meters. nothing else works. it's motor rotations
    //    auto constraints
    //        = frc::TrapezoidProfile<units::meters>::Constraints{ units::meters_per_second_t{ CONSTANT("TELESCOPE_V_TPS") },
    //                                                             units::meters_per_second_t{ CONSTANT("TELESCOPE_A_TPS2") }
    //                                                                 / units::second_t{ 1 } };
    //    auto goal    = frc::TrapezoidProfile<units::meters>::State{ units::meter_t{ pos }, 0_fps };
    //    auto initial = frc::TrapezoidProfile<units::meters>::State{ units::meter_t{ GetPosition() },
    //                                                                units::meters_per_second_t{
    //                                                                    m_TelescopeMotor->GetVelocity() * flip } };
    //
    //    auto trapezoidalProfile = frc::TrapezoidProfile<units::meters>(constraints, goal, initial);
    //
    //    auto setpoint = trapezoidalProfile.Calculate(units::second_t{ 0.02 });
    //
    //    m_MotorRequest.Position = setpoint.position.value() * flip;

    m_MotorRequest.Position = pos * -1 * CONSTANT("TELESCOPE_GEAR_RATIO");

    if (GetPosition() < GetSetpoint())
    {
        m_MotorRequest.FeedForward = CONSTANT("TELESCOPE_FF");
    }
    else
    {
        m_MotorRequest.FeedForward = 0;
    };
}

double Telescope::GetSetpoint()
{
    return m_MotorRequest.Position * -1 / CONSTANT("TELESCOPE_GEAR_RATIO");
}

bool Telescope::AtTarget()
{
    return fabs(GetPosition() - GetSetpoint()) < CONSTANT("TELESCOPE_TOLERANCE");
}

double Telescope::GetPosition()
{
    return m_TelescopeMotor->GetPosition() * -1 / CONSTANT("TELESCOPE_GEAR_RATIO");
}

void Telescope::UpdatePID(double armExt)
{
    // todo
}

void Telescope::ResetConstants()
{
    m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_DOWN_P"),
                             CONSTANT("TELESCOPE_DOWN_I"),
                             CONSTANT("TELESCOPE_DOWN_D"),
                             CONSTANT("TELESCOPE_DOWN_F"));
    m_TelescopeMotor->SetMotionMagic(CONSTANT("TELESCOPE_DOWN_V"), CONSTANT("TELESCOPE_DOWN_A"));
}

void Telescope::Handle()
{
    m_TelescopeMotor->Set(m_MotorRequest);
}

void Telescope::UsePIDSet(Telescope::PIDSet set)
{
    if (set != m_PrevPIDSet)
    {
        switch (set)
        {
        case EXTENDING :
            m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_UP_P"),
                                     CONSTANT("TELESCOPE_UP_I"),
                                     CONSTANT("TELESCOPE_UP_D"),
                                     CONSTANT("TELESCOPE_UP_F"));
            m_TelescopeMotor->SetMotionMagic(CONSTANT("TELESCOPE_UP_V"), CONSTANT("TELESCOPE_UP_A"));
            printf("UP telescope constants\n");

            break;
        case RETRACTING :
            m_TelescopeMotor->SetPID(CONSTANT("TELESCOPE_DOWN_P"),
                                     CONSTANT("TELESCOPE_DOWN_I"),
                                     CONSTANT("TELESCOPE_DOWN_D"),
                                     CONSTANT("TELESCOPE_DOWN_F"));
            m_TelescopeMotor->SetMotionMagic(CONSTANT("TELESCOPE_DOWN_V"), CONSTANT("TELESCOPE_DOWN_A"));
            printf("DOWN telescope constants\n");

            break;
        default :
            break;
        }
    }

    m_PrevPIDSet = set;
}

void Telescope::BrakeMode(bool brakeMode)
{
    if (brakeMode)
    {
        m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::NeutralMode::BRAKE);
    }
    else
    {
        m_TelescopeMotor->SetNeutralMode(CowLib::CowMotorController::NeutralMode::COAST);
    }
}
