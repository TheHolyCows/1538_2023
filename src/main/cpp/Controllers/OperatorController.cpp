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

    bot->GetArm()->InvertArm(m_CB->GetOperatorButton(SW_ORIENT));

    if (m_CB->GetOperatorButton(BT_CONE))
    {
        bot->SetArmState(ARM_IN, CG_CONE);
    }
    else if (m_CB->GetOperatorButton(BT_CUBE))
    {
        bot->SetArmState(ARM_IN, CG_CUBE);
    }
    else if (bot->GetArm()->GetArmState() == ARM_IN)
    {
        // neither button is pressed, we default to off state
        // this is really an idle state that is only reachable
        // when going from holding the intake buttons to releasing them
        bot->SetArmState(ARM_NONE, CG_NONE);
    }

    // only ARM_IN will change cargo within Arm subsystem code
    // safe to put CG_NONE for the remaining states, it will be ignored
    if (m_CB->GetOperatorButton(BT_STOW))
    {
        bot->SetArmState(ARM_STOW, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_L3))
    {
        bot->SetArmState(ARM_L3, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_L2))
    {
        bot->SetArmState(ARM_L2, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_GND))
    {
        bot->SetArmState(ARM_GND, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_SCORE))
    {
        bot->SetArmState(ARM_SCORE, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_HUMAN))
    {
        bot->SetArmState(ARM_HUMAN, CG_NONE);
    }

    // manual control - not sure if this will be final implementation
    // deadband is higher because I really don't want to accidentally hit it
    if (fabs(m_CB->GetOperatorAxis(1)) > 0.10) //CONSTANT("STICK_DEADBAND"))
    {
        bot->SetArmState(ARM_MANUAL, CG_NONE);
        if (m_CB->GetOperatorAxis(2) > 0.85)
        {
            bot->GetArm()->ManualPosition(m_CB->GetOperatorAxis(1), true);
        }
        else
        {
            bot->GetArm()->ManualPosition(m_CB->GetOperatorAxis(1), false);
        }
    }
    else
    {
        bot->GetArm()->ManualPosition(0, false);
    }

    // Moved to CowRobot.cpp handle
    // bot->ArmSM();
}
