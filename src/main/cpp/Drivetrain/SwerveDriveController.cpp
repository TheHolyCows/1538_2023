#include "SwerveDriveController.h"

SwerveDriveController::SwerveDriveController(SwerveDrive &drivetrain)
    : m_Drivetrain(drivetrain),
      m_Gyro(*CowPigeon::GetInstance()),
      m_HeadingLocked(false),
      m_TargetHeading(0)
{
    m_ExponentialFilter = std::make_unique<CowLib::CowExponentialFilter>(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    m_HeadingPIDController
        = std::make_unique<frc2::PIDController>(CONSTANT("HEADING_P"), CONSTANT("HEADING_I"), CONSTANT("HEADING_D"));
    m_HeadingPIDController->EnableContinuousInput(0, 360);
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

    // if (m_CB->GetLeftDriveStickAxis(3) > 0.85)
    // {
    //     if (m_EvasiveSwerveWheel == NONE)
    //     {
    //         double stickAngle = atan2(m_CB->GetLeftDriveStickAxis(1), m_CB->GetLeftDriveStickAxis(0)) * 180 / M_PI;
    //         stickAngle        = (stickAngle < 0) ? (360 + stickAngle) : stickAngle;

    //         double robotOrientedAngle = bot->GetGyro()->GetYawDegrees() + stickAngle;

    //         // set wheel based on quadrant
    //         if (robotOrientedAngle >= 0 && robotOrientedAngle < 90)
    //         {
    //             m_EvasiveSwerveWheel = FRONT_LEFT;
    //         }
    //         else if (robotOrientedAngle >= 90 && robotOrientedAngle < 180)
    //         {
    //             m_EvasiveSwerveWheel = FRONT_RIGHT;
    //         }
    //         else if (robotOrientedAngle >= 180 && robotOrientedAngle < 270)
    //         {
    //             m_EvasiveSwerveWheel = BACK_RIGHT;
    //         }
    //         else if (robotOrientedAngle >= 270 && robotOrientedAngle < 360)
    //         {
    //             m_EvasiveSwerveWheel = BACK_LEFT;
    //         }
    //         else
    //         {
    //             CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "Invalid angle for quick turn");
    //         }
    //     }

    //     const double wb = CONSTANT("WHEEL_BASE");

    //     switch (m_EvasiveSwerveWheel)
    //     {
    //     case FRONT_LEFT :
    //         centerOfRotationX = wb;
    //         centerOfRotationY = wb;
    //         break;
    //     case FRONT_RIGHT :
    //         centerOfRotationX = wb;
    //         centerOfRotationY = -wb;
    //         break;
    //     case BACK_LEFT :
    //         centerOfRotationX = -wb;
    //         centerOfRotationY = wb;
    //         break;
    //     case BACK_RIGHT :
    //         centerOfRotationX = -wb;
    //         centerOfRotationY = -wb;
    //         break;
    //     case NONE :
    //         break;
    //     }
    // }
    // else
    // {
    //     m_EvasiveSwerveWheel = NONE;
    // }

    double omega = 0;

    frc::SmartDashboard::PutNumber("rotation axis", rotation);

    if (fabs(rotation) > CONSTANT("STICK_DEADBAND"))
    {
        omega           = ProcessDriveAxis(rotation, CONSTANT("DESIRED_MAX_ANG_VEL"), false);
        m_HeadingLocked = false;
    }
    else
    {
        double heading = m_Drivetrain.GetPoseRot();
        if (fabs(heading - m_PrevHeading) < CONSTANT("HEADING_PID_THRESHOLD"))
        {
            frc::SmartDashboard::PutNumber("heading lock setpoint", m_TargetHeading);

            if (!m_HeadingLocked)
            {
                m_TargetHeading = heading;
                m_HeadingLocked = true;
            }

            omega = m_HeadingPIDController->Calculate(m_Gyro.GetYawDegrees(), m_TargetHeading);
            frc::SmartDashboard::PutNumber("heading pid error", m_HeadingPIDController->GetPositionError());
            frc::SmartDashboard::PutNumber("heading pid output", omega);
        }
    }

    frc::SmartDashboard::PutNumber("heading locked", m_HeadingLocked);
    frc::SmartDashboard::PutNumber("omega deg / sec", omega);

    m_Drivetrain.SetVelocity(ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false),
                             ProcessDriveAxis(y, CONSTANT("DESIRED_MAX_SPEED"), false),
                             omega,
                             fieldRelative,
                             centerOfRotationX,
                             centerOfRotationY);

    m_PrevHeading = m_Drivetrain.GetPoseRot();
}

void SwerveDriveController::LockHeadingToScore(double x, double y, bool armFlipped)
{
    double omega = 0;

    if (armFlipped)
    {
        m_TargetHeading = 180;
    } else {
        m_TargetHeading = 0;
    }
    m_HeadingLocked = true;

    // idk if this makes a difference but should ensure no weird PID to past heading things
    m_PrevHeading = m_TargetHeading;

    omega = m_HeadingPIDController->Calculate(m_Gyro.GetYawDegrees(), m_TargetHeading);

    m_Drivetrain.SetVelocity(ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false),
                             ProcessDriveAxis(y, CONSTANT("DESIRED_MAX_SPEED"), false),
                             omega,
                             fieldRelative,
                             0,
                             0);

    m_TargetHeading = heading;
    m_HeadingLocked = true;
}

void SwerveDriveController::CubeAlign(double x)
{
    double y     = 0;
    double omega = 0;

    if (!Vision::GetInstance()->CubeYawAligned())
    {
        omega = Vision::GetInstance()->CubeYawPID();
    }
    else
    {
        m_TargetHeading = m_Drivetrain.GetPoseRot();
        y               = Vision::GetInstance()->CubeYPID();
    }

    x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

    m_Drivetrain.SetVelocity(x, y, omega, false, 0, 0, true);

    m_HeadingLocked = false;
}

void SwerveDriveController::ConeAlign(double x, double yInput, bool armFlipped)
{
    double y     = 0;
    double omega = 0;

    // TODO: confirm this logic is correct
    double targetHeading = armFlipped ? 0 : 180;

    if (fabs(m_Gyro.GetYawDegrees() - targetHeading) < CONSTANT("CONE_YAW_THRESHOLD"))
    {
        if (!Vision::GetInstance()->ConeYAligned())
        {
            y = Vision::GetInstance()->ConeYPID();
        }
    }
    else
    {
        omega = m_HeadingPIDController->Calculate(m_Gyro.GetYawDegrees(), targetHeading);
    }

    // Override if yInput is above override threshold
    if (fabs(yInput) > CONSTANT("CONE_Y_OVERRIDE_THRESHOLD"))
    {
        y = ProcessDriveAxis(yInput, CONSTANT("DESIRED_MAX_SPEED"), false);
    }

    x = ProcessDriveAxis(x, CONSTANT("DESIRED_MAX_SPEED"), false);

    m_Drivetrain.SetVelocity(x, y, omega, false, 0, 0, true);
}

double SwerveDriveController::ProcessDriveAxis(double input, double scale, bool reverse)
{
    return m_ExponentialFilter->Filter(CowLib::Deadband(input, CONSTANT("STICK_DEADBAND"))) * scale
           * (reverse ? -1 : 1);
}