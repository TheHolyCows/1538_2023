//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// CowControlBoard.h
// author: jon-bassi
//==================================================

#pragma once /* __COW_CONTROL_BOARD_H__ */

#include "ButtonMap.h"
#include "GenericControlBoard.h"

#include <frc/Joystick.h>

class CowControlBoard : public GenericControlBoard
{
private:
    frc::Joystick *m_DriverControlStick;
    frc::Joystick *m_OperatorPanel;
    frc::Joystick *m_OperatorControlStick;

    bool m_PreviousAuto;
    bool m_PreviousReset;

public:
    CowControlBoard();

    bool GetAutoSelectButton() override;
    bool GetConstantsResetButton() override;
    bool GetRobotRelativeButton() override;
    bool GetVisionTargetButton() override;

    bool GetDriveButton(int) override;
    double GetDriveAxis(int) override;

    double GetLeftDriveStickX() override;
    double GetLeftDriveStickY() override;

    double GetRightDriveStickX() override;
    double GetRightDriveStickY() override;

    bool GetOperatorButton(int) override;
    double GetOperatorAxis(int) override;

    ~CowControlBoard();
};