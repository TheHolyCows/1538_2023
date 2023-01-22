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
    public:
        enum ControlMode
        {
            NO_CONTROL_MODE,
            OUT,
            POSITION,
            VELOCITY,
            MOTION_MAGIC,
            FOLLOWER
        };

        enum ControlMethod
        {
            NO_CONTROL_METHOD,
            DUTY_CYCLE,
            VOLTAGE,
            TORQUE
        };

    private:
        ctre::phoenixpro::hardware::TalonFX *m_Talon;
        ctre::phoenixpro::controls::ControlRequest *m_ControlRequest;
        ControlMode m_ControlMode;
        ControlMethod m_ControlMethod;
        double m_Offset;

        ctre::phoenixpro::controls::ControlRequest *TranslateControlMode(ControlMode mode,
                                                                         ControlMethod method,
                                                                         double value,
                                                                         double feedforward     = 0,
                                                                         bool useFOC            = true,
                                                                         bool overrideBrakeMode = false);

    public:
        CowMotorController(int id);
        ~CowMotorController();

        ctre::phoenixpro::hardware::TalonFX *GetInternalTalon();

        void ApplyConfig(std::variant<ctre::phoenixpro::configs::TalonFXConfiguration,
                                      ctre::phoenixpro::configs::Slot0Configs,
                                      ctre::phoenixpro::configs::MotionMagicConfigs> config);

        // out, position, velocity, motion magic, follower
        // position, velocity, torque
        // ctre::phoenixpro::controls::PositionDutyCycle

        void SetControl(ctre::phoenixpro::controls::ControlRequest &control);

        void SetControlMode(ControlMode mode);
        void SetControlMethod(ControlMethod method);

        double GetPosition();
        double GetVelocity();

        int SetSensorPosition(double turns);

        void Set(double value);

        void SetPID(double p, double i, double d, double f = 0.0);
        void SetMotionMagic(double velocity, double acceleration);

        void SetInverted(bool inverted);

        void GetPIDData(double *setpoint, double *procVar, double *P, double *I, double *D);
        void GetLogData(double *temp, double *encoderCt, bool *isInverted);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
