#include "OperatorController.h"

#include "frc/TimedRobot.h"

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{
    bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickAxis(1),
                                     m_CB->GetLeftDriveStickAxis(2),
                                     m_CB->GetLeftDriveStickAxis(4),
                                     true);
}
