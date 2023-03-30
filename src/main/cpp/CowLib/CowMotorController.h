//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWMOTORCONTROLLER_H__
#define __COWLIB_COWMOTORCONTROLLER_H__

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
            // Percent of total motor output (-1 to 1)
            double PercentOut;

            double GetSetpoint() { return PercentOut; }

            ctre::phoenixpro::controls::DutyCycleOut ToControlRequest() { return { PercentOut }; }
        };

        struct VoltageOutput
        {
            // Voltage to set the motor to
            double Voltage;

            double GetSetpoint() { return Voltage; }

            ctre::phoenixpro::controls::VoltageOut ToControlRequest() { return { units::volt_t{ Voltage } }; }
        };

        struct TorqueCurrentOutput
        {
            // Motor current in amps
            double Current;

            // Max absolute output of the motor controller (0 to 1)
            double MaxOutput = 1;

            // Deadband in amps. Deadband of 1 means the motor will stop quickly when set to 0
            double Deadband = 1;

            double GetSetpoint() { return Current; }

            ctre::phoenixpro::controls::TorqueCurrentFOC ToControlRequest()
            {
                return { units::ampere_t{ Current }, MaxOutput, units::ampere_t{ Deadband }, false };
            }
        };

        struct PositionPercentOutput
        {
            // Position in turns
            double Position;

            // Feedforward in percent of total motor output (-1 to 1)
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::PositionDutyCycle ToControlRequest()
            {
                return { units::turn_t{ Position }, true, FeedForward, 0, false };
            }
        };

        struct PositionVoltage
        {
            // Position in turns
            double Position;

            // Feedforward in volts
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::PositionVoltage ToControlRequest()
            {
                return { units::turn_t{ Position }, true, units::volt_t{ FeedForward }, 0, false };
            }
        };

        struct PositionTorqueCurrent
        {
            // Position in turns
            double Position;

            // Feedforward in amps
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::PositionTorqueCurrentFOC ToControlRequest()
            {
                return { units::turn_t{ Position }, units::ampere_t{ FeedForward }, 0, false };
            }
        };

        struct VelocityPercentOutput
        {
            // Velocity in turns per second
            double Velocity;

            // Feedforward in percent of total motor output (-1 to 1)
            double FeedForward = 0;

            double GetSetpoint() { return Velocity; }

            ctre::phoenixpro::controls::VelocityDutyCycle ToControlRequest()
            {
                return { units::turns_per_second_t{ Velocity }, true, FeedForward, 0, false };
            }
        };

        struct VelocityVoltage
        {
            // Velocity in turns per second
            double Velocity;

            // Feedforward in volts
            double FeedForward = 0;

            double GetSetpoint() { return Velocity; }

            ctre::phoenixpro::controls::VelocityVoltage ToControlRequest()
            {
                return { units::turns_per_second_t{ Velocity }, true, units::volt_t{ FeedForward }, 0, false };
            }
        };

        struct VelocityTorqueCurrent
        {
            // Velocity in turns per second
            double Velocity;

            // Feedforward in amps
            double FeedForward = 0;

            double GetSetpoint() { return Velocity; }

            ctre::phoenixpro::controls::VelocityTorqueCurrentFOC ToControlRequest()
            {
                return { units::turns_per_second_t{ Velocity }, units::ampere_t{ FeedForward }, 0, false };
            }
        };

        struct MotionMagicPercentOutput
        {
            // Position in turns
            double Position;

            // Feedforward in percent of total motor output (-1 to 1)
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::MotionMagicDutyCycle ToControlRequest()
            {
                return { units::turn_t{ Position }, true, FeedForward, 0, false };
            }
        };

        struct MotionMagicVoltage
        {
            // Position in turns
            double Position;

            // Feedforward in volts
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::MotionMagicVoltage ToControlRequest()
            {
                return { units::turn_t{ Position }, true, units::volt_t{ FeedForward }, 0, false };
            }
        };

        struct MotionMagicTorqueCurrent
        {
            // Position in turns
            double Position;

            // Feedforward in amps
            double FeedForward = 0;

            double GetSetpoint() { return Position; }

            ctre::phoenixpro::controls::MotionMagicTorqueCurrentFOC ToControlRequest()
            {
                return { units::turn_t{ Position }, FeedForward, 0, false };
            }
        };

        struct Follower
        {
            // ID of the motor to follow
            int LeaderID;

            // Whether to invert the motor against the leader
            bool Invert = false;

            ctre::phoenixpro::controls::Follower ToControlRequest() { return { LeaderID, Invert }; }
        };

        enum NeutralMode
        {
            COAST,
            BRAKE
        };

        CowMotorController(int id, std::string bus = "cowbus");

        ~CowMotorController();

        void Set(std::variant<PercentOutput,
                              VoltageOutput,
                              PositionPercentOutput,
                              PositionVoltage,
                              VelocityPercentOutput,
                              VelocityVoltage,
                              MotionMagicPercentOutput,
                              MotionMagicVoltage> request);

        void
        Set(std::variant<TorqueCurrentOutput, PositionTorqueCurrent, VelocityTorqueCurrent, MotionMagicTorqueCurrent>
                request);

        void Set(Follower request);

        void UseFOC(bool useFOC);
        void OverrideBrakeMode(bool overrideBrakeMode);

        void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                      ctre::phoenixpro::configs::Slot0Configs,
                                      ctre::phoenixpro::configs::MotionMagicConfigs,
                                      ctre::phoenixpro::configs::MotorOutputConfigs> config);

        double GetPosition();
        double GetVelocity();
        double GetTorqueCurrent();
        double GetRefreshTorqueCurrent();

        int SetSensorPosition(double turns);

        void SetNeutralMode(NeutralMode mode);
        NeutralMode GetNeutralMode();

        void SetPID(double p, double i, double d, double f = 0.0);
        void SetMotionMagic(double velocity, double acceleration);

        void SetInverted(bool inverted);

        ctre::phoenixpro::hardware::TalonFX *GetInternalTalon();

        void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D);
        void GetLogData(double *temp, double *encoderCt, bool *isInverted);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
