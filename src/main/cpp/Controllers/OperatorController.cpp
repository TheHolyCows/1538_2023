#include "OperatorController.h"

#include <iostream>

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;

    m_EvasiveSwerveWheel = NONE;
}

void OperatorController::Handle(CowRobot *bot)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    if (m_CB->GetLeftDriveStickAxis(3) > 0.85)
    {
        if (m_EvasiveSwerveWheel == NONE)
        {
            const double robotAngle = bot->GetGyro()->GetYaw();
            double stickAngle = atan2(m_CB->GetLeftDriveStickAxis(1), m_CB->GetLeftDriveStickAxis(0)) * 180 / M_PI;
            stickAngle        = (stickAngle < 0) ? (360 + stickAngle) : stickAngle;
            const double robotOrientedAngle = robotAngle + stickAngle;

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

        const double wb = CONSTANT("WHEEL_BASE") / 2.0;

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

    // Swerve controls for xbox controller
    auto chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(
        m_CB->GetLeftDriveStickAxis(0) * CONSTANT("SWERVE_MAX_SPEED"),
        -m_CB->GetLeftDriveStickAxis(1) * CONSTANT("SWERVE_MAX_SPEED"),
        m_CB->GetLeftDriveStickAxis(4),
        bot->GetGyro()->GetYaw());

    // Can use triggers to do funny turn thing
    bot->GetDrivetrain()->SetVelocity(chassisSpeeds, centerOfRotationX, centerOfRotationY);
}
