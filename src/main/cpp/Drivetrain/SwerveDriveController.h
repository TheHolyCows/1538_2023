#pragma once

#include "../CowLib/CowExponentialFilter.h"
#include "../CowPigeon.h"
#include "../Subsystems/Vision.h"
#include "SwerveDrive.h"

#include <frc/controller/ProfiledPIDController.h>
#include <memory>

class SwerveDriveController
{
public:
    SwerveDriveController(SwerveDrive &drivetrain);
    ~SwerveDriveController() = default;

    void Drive(double x, double y, double rotation, bool fieldRelative);

    void CubeAlign(double x);

    void ConeAlign(double x, double yInput);

    void LockHeading(double x, double y, bool useRawInputs=false);

    void ResetHeadingLock();

    void ResetConstants();

private:
    double ProcessDriveAxis(double input, double scale, bool reverse);

    SwerveDrive &m_Drivetrain;

    CowPigeon &m_Gyro;

    std::unique_ptr<CowLib::CowExponentialFilter> m_ExponentialFilter;

    std::unique_ptr<frc::ProfiledPIDController<units::meters>> m_HeadingPIDController;

    bool m_HeadingLocked;
    double m_TargetHeading;
    double m_PrevHeading;
};