#pragma once

#include "../CowLib/CowExponentialFilter.h"
#include "../CowPigeon.h"
#include "../Subsystems/Vision.h"
#include "SwerveDrive.h"

#include <frc/controller/PIDController.h>
#include <memory>

class SwerveDriveController
{
public:
    SwerveDriveController(SwerveDrive &drivetrain);
    ~SwerveDriveController() = default;

    void Drive(double x, double y, double rotation, bool fieldRelative);

    void CubeAlign(double x);

    void ConeAlign(double x, double yInput, bool armFlipped);

    void LockHeadingToScore(double x, double y, bool armFlipped);

    void ResetConstants();

private:
    double ProcessDriveAxis(double input, double scale, bool reverse);

    SwerveDrive &m_Drivetrain;

    CowPigeon &m_Gyro;

    std::unique_ptr<CowLib::CowExponentialFilter> m_ExponentialFilter;

    std::unique_ptr<frc2::PIDController> m_HeadingPIDController;

    bool m_HeadingLocked;
    double m_TargetHeading;
    double m_PrevHeading;
};