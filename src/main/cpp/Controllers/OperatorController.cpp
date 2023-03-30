#include "OperatorController.h"

OperatorController::OperatorController(GenericControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{
    bool inverted = !m_CB->GetOperatorButton(SW_ORIENT);
    bot->GetArm()->InvertArm(inverted);
    Vision::GetInstance()->SetInverted(inverted);

    if (m_CB->GetDriveAxis(2) > 0.8 && m_CB->GetDriveAxis(6) > 0.8)
    {
        bot->GetDrivetrain()->SetLocked(true);
        bot->GetDrivetrain()->SetVelocity(0, 0, 0);
    }
    else
    {
        bot->GetDrivetrain()->SetLocked(false);
    }

    if (m_CB->GetVisionTargetButton())
    {
        // switch (bot->GetArm()->GetArmCargo())
        // {
        // case CG_CONE :
        //     bot->GetDriveController()->ConeAlign(m_CB->GetLeftDriveStickY(), m_CB->GetLeftDriveStickX());
        //     break;
        // case CG_CUBE :
        //     bot->GetDriveController()->CubeAlign(m_CB->GetLeftDriveStickY());
        //     break;
        // default :
        //     break;
        // }
    }
    else if (m_CB->GetDriveAxis(3) > 0.8) // Align heading
    {
        bot->GetDriveController()->LockHeading(m_CB->GetLeftDriveStickY(), m_CB->GetLeftDriveStickX());
    }
    else
    {
        // standard drive with field/bot relative option
        bot->GetDriveController()->Drive(m_CB->GetLeftDriveStickY(),
                                         m_CB->GetLeftDriveStickX(),
                                         m_CB->GetRightDriveStickX() * -1,
                                         true);
    }

    // ARM STATES
    // if (m_CB->GetOperatorButton(BT_WRIST_FLIP))
    // {
    //     if (!m_WristFlipCheck)
    //     {
    //         bot->GetArm()->FlipWristState();
    //     }
    //     m_WristFlipCheck = true;
    // }
    // else
    // {
    m_WristFlipCheck = false;
    // }

    // New claw logic
    if (m_CB->GetOperatorButton(BT_CONE))
    {
        // alternatively this can be put in the ARM_GND button, however, that would cause
        // issues going to ground with a game piece in the intake
        bot->AllowAutoStow();
        bot->GetArm()->SetClawState(CLAW_INTAKE);
        bot->GetArm()->SetArmCargo(CG_CONE);
        Vision::GetInstance()->SetCargo(CG_CONE);
        bot->GetArm()->UpdateClawState();
    }
    else if (m_CB->GetOperatorButton(BT_CUBE))
    {
        bot->AllowAutoStow();
        bot->GetArm()->SetClawState(CLAW_INTAKE);
        bot->GetArm()->SetArmCargo(CG_CUBE);
        Vision::GetInstance()->SetCargo(CG_CUBE);
        bot->GetArm()->UpdateClawState();
    }
    else if (m_CB->GetOperatorButton(BT_SCORE))
    {
        bot->GetArm()->SetClawState(CLAW_EXHAUST);
        bot->GetArm()->UpdateClawState();
    }
    else
    {
        bot->GetArm()->SetClawState(CLAW_OFF);
        bot->GetArm()->UpdateClawState();
    }

    // only ARM_IN will change cargo within Arm subsystem code
    // safe to put CG_NONE for the remaining states, it will be ignored
    if (m_CB->GetDriveAxis(3) > 0.8) // driver stow
    {
        bot->SetArmState(ARM_DRIVER_STOW, CG_NONE);
    }
    if (m_CB->GetOperatorButton(BT_STOW))
    {
        bot->SetArmState(ARM_DRIVER_STOW, CG_NONE);
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
    else if (m_CB->GetOperatorButton(BT_HUMAN))
    {
        bot->SetArmState(ARM_HUMAN, CG_NONE);
    }
    else if (m_CB->GetOperatorButton(BT_WRIST_FLIP))
    {
        bot->SetArmState(ARM_UP, CG_NONE);
    }

    // manual control - not sure if this will be final implementation
    // deadband is higher because I really don't want to accidentally hit it
    if (fabs(m_CB->GetOperatorAxis(1)) > 0.10) //CONSTANT("STICK_DEADBAND"))
    {
        bot->GetArm()->UseManualControl(true);

        if (m_CB->GetOperatorAxis(2) > 0.8)
        {
            bot->GetArm()->ManualPosition(m_CB->GetOperatorAxis(1), false);
        }
        else
        {
            bot->GetArm()->ManualPosition(m_CB->GetOperatorAxis(1), true);
        }
    }
}
