#include "OperatorController.h"

#include <iostream>

OperatorController::OperatorController(CowControlBoard *controlboard)
    : m_CB(controlboard)
{
    m_TrackingCooldownTimer = 0.0;

    m_EvasiveSwerveWheel = NONE;
}

void OperatorController::Handle(CowRobot *bot)
{
}
