//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_BASE_H__
#define __COW_BASE_H__

#include "Autonomous/AutoModes.h"
#include "Controllers/AutoModeController.h"
#include "Controllers/OperatorController.h"
#include "CowConstants.h"
#include "CowControlBoard.h"
#include "CowDisplay.h"
#include "CowLib/CowLib.h"
#include "CowLib/CowLogger.h"

#include <frc/TimedRobot.h>
#include <iostream>

class CowBase : public frc::TimedRobot
{
private:
    CowRobot *m_Bot;
    CowControlBoard *m_ControlBoard;
    OperatorController *m_OpController;
    AutoModeController *m_AutoController;
    CowConstants *m_Constants;
    CowDisplay *m_Display;

    int m_DisabledCount = 0;

public:
    CowBase();
    ~CowBase() override;
    void RobotInit() override;
    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;
};

#endif
