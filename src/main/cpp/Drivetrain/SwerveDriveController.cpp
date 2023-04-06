#include "SwerveDriveController.h"

SwerveDriveController::SwerveDriveController(SwerveDrive &drivetrain)
    : m_Drivetrain(drivetrain),
      m_Gyro(*CowPigeon::GetInstance()),
      m_HeadingLocked(false),
      m_TargetHeading(0)
{
    m_ExponentialFilter = std::make_unique<CowLib::CowExponentialFilter>(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    m_HeadingPIDController = std::make_unique<frc::ProfiledPIDController<units::meters>>(
        CONSTANT("HEADING_P"),
        CONSTANT("HEADING_I"),
        CONSTANT("HEADING_D"),
        frc::TrapezoidProfile<units::meters>::Constraints{
            units::meters_per_second_t{ CONSTANT("HEADING_V") },
            units::meters_per_second_squared_t{ CONSTANT("HEADING_A") } });

    m_HeadingPIDController->EnableContinuousInput(units::meter_t{ 0 }, units::meter_t{ 360 });
}

void SwerveDriveController::ResetConstants()
{
    m_ExponentialFilter->Reset(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    m_HeadingPIDController->SetPID(CONSTANT("HEADING_P"), CONSTANT("HEADING_I"), CONSTANT("HEADING_D"));
}

void SwerveDriveController::Drive(double x, double y, double rotation, bool fieldRelative)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    double omega = 0;

    // frc::SmartDashboard::PutNumber("rotation axis", rotation);

    double heading = m_Gyro.GetYawDegrees();
    if (fabs(rotation) > CONSTANT("STICK_DEADBAND"))
    {
        omega           = ProcessDriveAxis(rotation, CONSTANT("DESIRED_MAX_ANG_VEL"), false);
        m_HeadingLocked = false;
    }
    else
    {
        // double heading = m_Drivetrain.GetPoseRot();
        if (fabs(heading - m_PrevHeading) < CONSTANT("HEADING_PID_THRESHOLD") && !m_HeadingLocked)
        {
            // frc::SmartDashboard::PutNumber("heading lock setpoint", m_TargetHeading);

            // if (!m_HeadingLocked)
            // {
            m_TargetHeading = m_PrevHeading;
            m_HeadingLocked = true;
            // }
        }
        else if (m_HeadingLocked)
        {
            omega = m_HeadingPIDController->Calculate(units::meter_t{ heading }, units::meter_t{ m_TargetHeading });
        }

        // omega = (fmod(heading, 360) - fmod(m_TargetHeading, 360)) * CONSTANT("HEADING_P");
        // frc::SmartDashboard::PutNumber("heading pid error", m_HeadingPIDController->GetPositionError());
        // frc::SmartDashboard::PutNumber("heading pid output", omega);
    }

    // frc::SmartDashboard::PutNumber("heading locked", m_HeadingLocked);
    // frc::SmartDashboard::PutNumber("omega deg / sec", omega);

    x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);
    y = ProcessDriveAxis(y, CONSTANT("DESIRED_MAX_SPEED"), false);
    if (x == 0 && y == 0 && fabs(rotation) < CONSTANT("STICK_DEADBAND"))
    {
        omega = 0;
    }

    m_Drivetrain.SetVelocity(x, y, omega, fieldRelative, centerOfRotationX, centerOfRotationY);

    m_PrevHeading = heading;
}

void SwerveDriveController::LockHeading(double x, double y, bool useRawInputs)
{
    double currentHeading = m_Gyro.GetYawDegrees();

    currentHeading = fmod(currentHeading, 360);
    if (currentHeading < 0)
    {
        currentHeading += 360;
    }

    if (currentHeading < 90 || currentHeading > 270)
    {
        m_TargetHeading = 0;
    }
    else
    {
        m_TargetHeading = 180;
    }

    m_HeadingLocked = true;

    // idk if this makes a difference but should ensure no weird PID to past heading things
    m_PrevHeading = m_TargetHeading;

    double omega = m_HeadingPIDController->Calculate(units::meter_t{ m_Gyro.GetYawDegrees() },
                                                     units::meter_t{ m_TargetHeading });

    if (!useRawInputs)
    {
        x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);
        y = ProcessDriveAxis(y, CONSTANT("DESIRED_MAX_SPEED"), false);
    }

    m_Drivetrain.SetVelocity(x, y, omega, true, 0, 0);
}

void SwerveDriveController::CubeAlign(double x)
{
    // Check if heading is aligned
    if (fabs(fmod(m_Gyro.GetYawDegrees(), 180)) > CONSTANT("HEADING_TOLERANCE"))
    {
        // If not, run the heading lock function
        LockHeading(x, 0);
        return;
    }

    // Otherwise, continue with vision alignment

    double y = Vision::GetInstance()->CubeYPID();

    x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

    m_Drivetrain.SetVelocity(x, y, 0, true, 0, 0, true);
}

void SwerveDriveController::ConeAlign(double x, double yInput)
{
    // Check if heading is aligned
    if (fabs(fmod(m_Gyro.GetYawDegrees(), 180)) > CONSTANT("HEADING_TOLERANCE"))
    {
        // If not, run the heading lock function
        LockHeading(x, yInput);
        return;
    }

    // Otherwise, continue with vision alignment
    double y = Vision::GetInstance()->ConeYPID();

    // Override if yInput is above override threshold
    if (fabs(yInput) < CONSTANT("CONE_Y_OVERRIDE_THRESHOLD"))
    {
        y = ProcessDriveAxis(yInput, CONSTANT("DESIRED_MAX_SPEED"), false);
    }

    x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

    m_Drivetrain.SetVelocity(x, y, 0, true, 0, 0, true);
}

void SwerveDriveController::ResetHeadingLock()
{
    m_HeadingLocked = false;
    m_TargetHeading = m_Gyro.GetYawDegrees();
}

double SwerveDriveController::ProcessDriveAxis(double input, double scale, bool reverse)
{
    return m_ExponentialFilter->Filter(CowLib::Deadband(input, CONSTANT("STICK_DEADBAND"))) * scale
           * (reverse ? -1 : 1);
}
