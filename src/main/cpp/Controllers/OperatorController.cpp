#include "OperatorController.h"

#include "frc/TimedRobot.h"

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;

    m_EvasiveSwerveWheel = NONE;

    m_ControllerExpFilter = new CowLib::CowExponentialFilter(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));

    m_PrevHeading   = 0;
    m_TargetHeading = 0;
    m_ThetaPID      = new frc2::PIDController(CONSTANT("HEADING_P"), CONSTANT("HEADING_I"), CONSTANT("HEADING_D"));
    m_ThetaPID->EnableContinuousInput(0, 360);
    m_HeadingLocked = false;
}

void OperatorController::Handle(CowRobot *bot)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    if (m_CB->GetLeftDriveStickAxis(3) > 0.85)
    {
        if (m_EvasiveSwerveWheel == NONE)
        {
            double stickAngle = atan2(m_CB->GetLeftDriveStickAxis(1), m_CB->GetLeftDriveStickAxis(0)) * 180 / M_PI;
            stickAngle        = (stickAngle < 0) ? (360 + stickAngle) : stickAngle;

            double robotOrientedAngle = bot->GetGyro()->GetYawDegrees() + stickAngle;

            // set wheel based on quadrant
            if (robotOrientedAngle >= 0 && robotOrientedAngle < 90)
            {
                m_EvasiveSwerveWheel = FRONT_LEFT;
            }
            else if (robotOrientedAngle >= 90 && robotOrientedAngle < 180)
            {
                m_EvasiveSwerveWheel = FRONT_RIGHT;
            }
            else if (robotOrientedAngle >= 180 && robotOrientedAngle < 270)
            {
                m_EvasiveSwerveWheel = BACK_RIGHT;
            }
            else if (robotOrientedAngle >= 270 && robotOrientedAngle < 360)
            {
                m_EvasiveSwerveWheel = BACK_LEFT;
            }
            else
            {
                CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR, "Invalid angle for quick turn");
            }
        }

        const double wb = CONSTANT("WHEEL_BASE");

        switch (m_EvasiveSwerveWheel)
        {
        case FRONT_LEFT :
            centerOfRotationX = wb;
            centerOfRotationY = wb;
            break;
        case FRONT_RIGHT :
            centerOfRotationX = wb;
            centerOfRotationY = -wb;
            break;
        case BACK_LEFT :
            centerOfRotationX = -wb;
            centerOfRotationY = wb;
            break;
        case BACK_RIGHT :
            centerOfRotationX = -wb;
            centerOfRotationY = -wb;
            break;
        case NONE :
            break;
        }
    }
    else
    {
        m_EvasiveSwerveWheel = NONE;
    }

    // Left trigger
    bool fieldRelative = m_CB->GetLeftDriveStickAxis(2) < 0.85;

    double omega = 0;

    frc::SmartDashboard::PutNumber("rotation axis", m_CB->GetLeftDriveStickAxis(4));

    if (fabs(m_CB->GetLeftDriveStickAxis(4)) > CONSTANT("STICK_DEADBAND"))
    {
        omega = m_ControllerExpFilter->Filter(
                    CowLib::Deadband(m_CB->GetLeftDriveStickAxis(4), CONSTANT("STICK_DEADBAND")))
                * CONSTANT("DESIRED_MAX_ANG_VEL") * -1;
        m_HeadingLocked = false;
    }
    else
    {
        double heading = bot->GetDrivetrain()->GetPoseRot();
        if (fabs(heading - m_PrevHeading) < CONSTANT("HEADING_PID_THRESHOLD"))
        {
            frc::SmartDashboard::PutNumber("heading lock setpoint", m_TargetHeading);

            if (!m_HeadingLocked)
            {
                m_TargetHeading = heading;
                m_HeadingLocked = true;
            }

            omega = m_ThetaPID->Calculate(bot->GetGyro()->GetYawDegrees(), m_TargetHeading);
            frc::SmartDashboard::PutNumber("heading pid error", m_ThetaPID->GetPositionError());
            frc::SmartDashboard::PutNumber("heading pid output", omega);
        }
    }

    frc::SmartDashboard::PutNumber("heading locked", m_HeadingLocked);
    frc::SmartDashboard::PutNumber("omega deg / sec", omega);

    bot->GetDrivetrain()->SetVelocity(
        m_ControllerExpFilter->Filter(CowLib::Deadband(m_CB->GetLeftDriveStickAxis(1), CONSTANT("STICK_DEADBAND")))
            * CONSTANT("DESIRED_MAX_SPEED") * -1,
        m_ControllerExpFilter->Filter(CowLib::Deadband(m_CB->GetLeftDriveStickAxis(0), CONSTANT("STICK_DEADBAND")))
            * CONSTANT("DESIRED_MAX_SPEED") * -1,
        omega,
        fieldRelative,
        centerOfRotationX,
        centerOfRotationY);

    m_PrevHeading = bot->GetDrivetrain()->GetPoseRot();
}

void OperatorController::ResetConstants()
{
    m_ControllerExpFilter->Reset(CONSTANT("STICK_EXPONENTIAL_MODIFIER"));
}