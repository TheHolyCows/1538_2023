#include "OperatorController.h"

#include "frc/TimedRobot.h"

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{
    if (m_CB->GetLeftDriveStickButton(5))
    {
        bot->GetDriveController()->AlignToScore(m_CB->GetLeftDriveStickAxis(1), Vision::GamePiece::CUBE);
    }
    else
    {
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickAxis(1),
                                         m_CB->GetLeftDriveStickAxis(0),
                                         m_CB->GetLeftDriveStickAxis(4),
                                         true);
    }
}
