//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_ROBOT_H__
#define __COW_ROBOT_H__

#include "Controllers/GenericController.h"
#include "CowConstants.h"
#include "CowGyro.h"
#include "CowLib/CowAlphaNum.h"
#include "CowLib/CowLogger.h"
#include "CowLib/CowMotorController.h"
#include "CowLib/CowTimer.h"
#include "CowLib/Utility.h"
#include "Drivetrain/CowWestcoast.h"
#include "Subsystems/Shooter.h"

#include <frc/BuiltInAccelerometer.h>
#include <frc/filter/LinearFilter.h>
#include <frc/PowerDistribution.h>
#include <math.h>

class CowRobot
{
public:
    // Drive Motors
    Drivetrain::CowWestcoast *m_Drivetrain;

private:
    int m_DSUpdateCount;

    GenericController *m_Controller = nullptr;

    Shooter *m_Shooter = nullptr;

    // gyro and accelerometers
    CowLib::CowGyro *m_Gyro;
    frc::Accelerometer *m_Accelerometer;
    frc::LinearFilter<double> m_ZFilter = frc::LinearFilter<double>::MovingAverage(12);
    double m_PrevZ;

    // PDP
    frc::PowerDistribution *m_PowerDistributionPanel;

    // display on rio
    CowLib::CowAlphaNum *m_LEDDisplay;

    double m_LeftDriveValue;
    double m_RightDriveValue;

    double m_PreviousGyroError;
    double m_PreviousDriveError;

    double m_MatchTime;
    double m_StartTime;

    void SetLeftMotors(double val);
    void SetRightMotors(double val);

public:
    CowRobot();
    void Reset();
    void GyroHandleCalibration();
    void GyroFinalizeCalibration();
    void SetController(GenericController *controller);
    void PrintToDS();
    bool DoVisionTracking(double speed, double threshold = 5.0);
    double GetDriveDistance();

    void DriveSpeedTurn(double speed, double turn, bool quickTurn);
    void DriveLeftRight(double leftDriveValue, double rightDriveValue);
    bool TurnToHeading(double heading);

    void QuickTurn(double turn);

    void StartTime();

    CowLib::CowAlphaNum *GetDisplay() { return m_LEDDisplay; }

    frc::PowerDistribution *GetPowerDistributionPanel() { return m_PowerDistributionPanel; }

    CowLib::CowGyro *GetGyro() { return CowLib::CowGyro::GetInstance(); }

    Shooter *GetShooter() { return m_Shooter; }

    void handle();
};

#endif