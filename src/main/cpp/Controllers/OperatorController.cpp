#include "OperatorController.h"

#include <iostream>

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;
}

void OperatorController::Handle(CowRobot *bot)
{
    // Swerve controls for xbox controller
    auto chassisSpeeds = CowLib::CowChassisSpeeds::FromFieldRelativeSpeeds(
        CowLib::Deadband(m_CB->GetLeftDriveStickAxis(0), CONSTANT("STICK_DEADBAND")) * CONSTANT("DESIRED_MAX_SPEED"),
        CowLib::Deadband(m_CB->GetLeftDriveStickAxis(1), CONSTANT("STICK_DEADBAND")) * CONSTANT("DESIRED_MAX_SPEED"),
        CowLib::Deadband(m_CB->GetLeftDriveStickAxis(4), CONSTANT("STICK_DEADBAND")),
        bot->GetGyro()->GetYaw());

    bot->GetDrivetrain()->SetVelocity(chassisSpeeds);
}
