#include "OperatorController.h"

#include <iostream>

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;

    m_QuickTurnWheel = NONE;
}

void OperatorController::Handle(CowRobot *bot)
{
    double centerOfRotationX = 0;
    double centerOfRotationY = 0;

    if (m_CB->GetLeftDriveStickAxis(3) > 0.85)
    {
        if (m_QuickTurnWheel == NONE)
        {
            const bool xPositive = m_CB->GetLeftDriveStickAxis(0) > 0.0;
            const bool yPositive = m_CB->GetLeftDriveStickAxis(1) > 0.0;

            if (xPositive && yPositive)
            {
                m_QuickTurnWheel = FRONT_LEFT;
            }
            else if (xPositive && !yPositive)
            {
                m_QuickTurnWheel = BACK_LEFT;
            }
            else if (!xPositive && yPositive)
            {
                m_QuickTurnWheel = FRONT_RIGHT;
            }
            else if (!xPositive && !yPositive)
            {
                m_QuickTurnWheel = BACK_RIGHT;
            }
        }

        const double wb = CONSTANT("WHEEL_BASE") / 2.0;

        switch (m_QuickTurnWheel)
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
        m_QuickTurnWheel = NONE;
    }

    // Swerve controls for xbox controller
    auto chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(m_CB->GetLeftDriveStickAxis(0),
                                                                           -m_CB->GetLeftDriveStickAxis(1),
                                                                           m_CB->GetLeftDriveStickAxis(4),
                                                                           bot->GetGyro()->GetYaw());

    // Can use triggers to do funny turn thing
    bot->GetDrivetrain()->SetVelocity(chassisSpeeds, centerOfRotationX, centerOfRotationY);
}
