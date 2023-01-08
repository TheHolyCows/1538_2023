//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COWLIB_COWMOTORCONTROLLER_H__
#define __COWLIB_COWMOTORCONTROLLER_H__

#include <ctre/Phoenix.h>
#include "ctre/phoenix/motorcontrol/StatorCurrentLimitConfiguration.h"

namespace CowLib
{
    class CowMotorController
    {
    public:
        enum CowNeutralMode
        {
            JUMPER,
            BRAKE,
            COAST,
        };

        enum CowControlMode
        {
            PERCENTVBUS,
            CURRENT,
            SPEED,
            POSITION,
            VOLTAGE,
            FOLLOWER,
            MOTIONPROFILE,
            MOTIONMAGIC
        };

        CowMotorController(int deviceNum);
        virtual ~CowMotorController();

        void SetClosedLoopError(int error);
        void SetNeutralMode(CowNeutralMode);
        enum CowNeutralMode GetNeutralMode();
        void SetControlMode(CowControlMode);
        enum CowControlMode GetControlMode();
        double GetPosition();
        void SetSensorPosition(double position);
        void SetPIDGains(double pGain, double iGain, double dGain, double fGain, double peakOutput);
        void SetMotionMagic(double accel, double velocity);
        void Set(double);
        void SetInverted(bool Value);
        void SetPeakCurrent(int amps, int ms);
        void SetStatorLimit(double limit, double threshold, double duration);
        double GetOutputCurrent();
        TalonFX *GetInternalMotor();

        // logging utility
        void GetLogData(double *temp, double *encoderCt, bool* isInverted);

    private:
        TalonFX *m_MotorController;
        int m_DeviceNum;
        enum CowNeutralMode m_CowNeutralMode;
        enum CowControlMode m_CowControlMode;

        ControlMode TranslateControlMode(enum CowControlMode);
    };
} // namespace CowLib

#endif /* __COWLIB_COWMOTORCONTROLLER_H__ */
