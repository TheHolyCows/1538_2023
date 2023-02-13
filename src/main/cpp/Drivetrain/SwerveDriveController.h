#pragma once

#include "../CowLib/CowExponentialFilter.h"
#include "../CowPigeon.h"
#include "SwerveDrive.h"

#include <frc/controller/PIDController.h>
#include <memory>

class SwerveDriveController
{
public:
    SwerveDriveController(SwerveDrive &drivetrain);
    ~SwerveDriveController() = default;

    void Drive(double x, double y, double rotation, bool fieldRelative);

    void ResetConstants();

private:
    SwerveDrive &m_Drivetrain;

    CowPigeon &m_Gyro;

    std::unique_ptr<CowLib::CowExponentialFilter> m_ExponentialFilter;

    std::unique_ptr<frc2::PIDController> m_HeadingPIDController;

    bool m_HeadingLocked;
    double m_TargetHeading;
    double m_PrevHeading;
};