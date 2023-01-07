#include "OperatorController.h"

#include <iostream>

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::handle(CowRobot *bot)
{
    // tracking button not pressed - standard driving
    if (!m_CB->GetSteeringButton(AUTO_AIM))
    {
        bot->m_Drivetrain->teleopDrive(m_CB->GetDriveStickY(), m_CB->GetDriveAxis(0), m_CB->GetSteeringButton(FAST_TURN));
    }

    if (m_CB->GetDriveButton(1))
    {
        std::cout << "holding button 1" << std::endl;
        bot->GetShooter()->SetSpeed(CONSTANT("SHOOTER_SPEED_LAUNCHPAD"));
    }
    else
    {
        bot->GetShooter()->SetSpeed(0);
    }
}
