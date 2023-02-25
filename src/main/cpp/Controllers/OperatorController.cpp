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

    // ARM STATES
    if (m_CB->GetOperatorButton(BT_WRIST_FLIP))
    {
        if (!m_WristFlipCheck)
        {
            bot->GetArm()->FlipWristState();
        }
        m_WristFlipCheck = true;
    }
    else
    {
        m_WristFlipCheck = false;
    }

    if (m_CB->GetOperatorButton(BT_CONE))
    {
        bot->SetArmState(ARM_IN, ST_CONE);
    }
    else if (m_CB->GetOperatorButton(BT_CUBE))
    {
        bot->SetArmState(ARM_IN, ST_CUBE);
    }
    else if (m_CB->GetOperatorButton(BT_STOW))
    {
        // only ARM_IN and ARM_NONE will change cargo within Arm subsystem code
        // safe to put none here, it will be ignored
        bot->SetArmState(ARM_STOW, ST_NONE);
    }

    bot->ArmSM();
}
