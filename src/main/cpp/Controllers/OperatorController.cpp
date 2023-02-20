#include "OperatorController.h"

OperatorController::OperatorController(GenericControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{
    // vision align
    if (m_CB->GetVisionTargetButton())
    {
        bot->GetDriveController()->AlignToScore(m_CB->GetLeftDriveStickY(), Vision::GamePiece::CUBE);
    }
    else
    {
        // standard drive with field/bot relative option
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickY(),
                                         m_CB->GetLeftDriveStickX(),
                                         m_CB->GetRightDriveStickX(),
                                         !m_CB->GetRobotRelativeButton());
    }

    // bot->ArmSM();
}
