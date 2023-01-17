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
            double stickAngle = atan2(m_CB->GetLeftDriveStickAxis(1), m_CB->GetLeftDriveStickAxis(0)) * 180 / M_PI;
            stickAngle        = (stickAngle < 0) ? (360 + stickAngle) : stickAngle;

            double robotOrientedAngle = bot->GetGyro()->GetYaw() + stickAngle;

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

    // Left trigger
    bool fieldRelative = m_CB->GetLeftDriveStickAxis(2) < 0.85;

    bot->GetDrivetrain()->SetVelocity(
        CowLib::Deadband(m_CB->GetLeftDriveStickAxis(0), CONSTANT("STICK_DEADBAND")) * CONSTANT("DESIRED_MAX_SPEED"),
        CowLib::Deadband(m_CB->GetLeftDriveStickAxis(1), CONSTANT("STICK_DEADBAND")) * CONSTANT("DESIRED_MAX_SPEED"),
        CowLib::Deadband(m_CB->GetRightDriveStickAxis(0), CONSTANT("STICK_DEADBAND")) * CONSTANT("DESIRED_MAX_ANG_VEL"),
        fieldRelative,
        centerOfRotationX,
        centerOfRotationY);
}
