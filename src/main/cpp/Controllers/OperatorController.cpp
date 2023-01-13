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
        m_CB->GetLeftDriveStickAxis(0) * CONSTANT("SWERVE_MAX_SPEED"),
        -m_CB->GetLeftDriveStickAxis(1) * CONSTANT("SWERVE_MAX_SPEED"),
        m_CB->GetLeftDriveStickAxis(4),
        bot->GetGyro()->GetYaw());

    bot->GetDrivetrain()->SetVelocity(chassisSpeeds);
}
