//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWMOTORCONTROLLER_H__
#define __COWLIB_COWMOTORCONTROLLER_H__

#include "ctre/phoenixpro/controls/MotionMagicDutyCycle.hpp"

#include <ctre/phoenixpro/TalonFX.hpp>
#include <variant>

namespace CowLib
{
    class CowMotorController
    {
    private:
        ctre::phoenixpro::hardware::TalonFX *m_Talon;
        double m_Setpoint;
        bool m_UseFOC;
        bool m_OverrideBrakeMode;

    public:
        struct PercentOutput
        {
            double PercentOut;

            double GetSetpoint() { return PercentOut; }

            ctre::phoenixpro::controls::DutyCycleOut ToControlRequest() { return { PercentOut }; }
        };

        struct PositionPercentOutput
        {
            double Position;
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::PositionDutyCycle ToControlRequest()
            {
                return { units::turn_t{ Position }, true, FeedForward, 0, false };
            }
        };

        struct VelocityPercentOutput
        {
            double Velocity;
            double FeedForward = 0;

            double GetSetpoint() { return Velocity; }

            ctre::phoenixpro::controls::VelocityDutyCycle ToControlRequest()
            {
                return { units::turns_per_second_t{ Velocity }, true, FeedForward, 0, false };
            }
        };

        struct MotionMagicPercentOutput
        {
            double Position;
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::MotionMagicDutyCycle ToControlRequest()
            {
                return { units::turn_t{ Position }, true, FeedForward, 0, false };
            }
        };

        struct Follower
        {
            int MasterID;
            bool Invert = false;

            ctre::phoenixpro::controls::Follower ToControlRequest() { return { MasterID, Invert }; }
        };

        CowMotorController(int id);

        ~CowMotorController();

        void Set(std::variant<PositionPercentOutput, PercentOutput> request);

        void UseFOC(bool useFOC);
        void OverrideBrakeMode(bool overrideBrakeMode);

        void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                      ctre::phoenixpro::configs::Slot0Configs,
                                      ctre::phoenixpro::configs::MotionMagicConfigs> config);

        double GetPosition();
        double GetVelocity();

        int SetSensorPosition(double turns);

        void SetPID(double p, double i, double d, double f = 0.0);
        void SetMotionMagic(double velocity, double acceleration);

        void SetInverted(bool inverted);

        ctre::phoenixpro::hardware::TalonFX *GetInternalTalon();

        void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D);
        void GetLogData(double *temp, double *encoderCt, bool *isInverted);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
