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
        double stickY = m_CB->GetLeftDriveStickY();

        bot->GetDriveController()->CubeAlign(m_CB->GetLeftDriveStickY());
        // TODO: re-enable this after testing

        //        switch (bot->GetArm()->GetArmCargo())
        //        {
        //        case ST_CONE :
        //            bot->GetDriveController()->ConeAlign(stickY);
        //            break;
        //        case ST_CUBE :
        //            bot->GetDriveController()->CubeAlign(stickY);
        //            break;
        //        case ST_NONE :
        //            break;
        //        }
    }
    else if (m_CB->GetDriveAxis(5) > 0.5)
    {
        bot->GetDriveController()->QuickTurn(m_CB->GetLeftDriveStickY(),
                                             m_CB->GetLeftDriveStickX(),
                                             m_CB->GetRightDriveStickX(),
                                             m_CB->GetRightDriveStickY(),
                                             !m_CB->GetRobotRelativeButton());
    }
    else
    {
        // standard drive with field/bot relative option
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickY(),
                                         m_CB->GetLeftDriveStickX(),
                                         m_CB->GetRightDriveStickX(),
                                         !m_CB->GetRobotRelativeButton());
    }

    // FORCE VISION AND GYRO RESET
    if (m_CB->GetDriveAxis(2) && m_CB->GetDriveAxis(3) && m_CB->GetDriveAxis(5) && m_CB->GetDriveAxis(6))
    {
        std::optional<Vision::BotPoseResult> botpose = Vision::GetInstance()->GetBotPose();
        if (botpose.has_value())
        {
            CowPigeon::GetInstance()->SetYaw((*botpose).pose.Rotation().Degrees().value());
            bot->GetDrivetrain()->ResetOdometry((*botpose).pose);
        }
    }

    // bot->ArmSM();
}
