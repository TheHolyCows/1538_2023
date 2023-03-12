//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// CowGamepad.h
// author: jon-bassi
//==================================================

#pragma once /* __COW_GAMEPAD_H__ */

#include "GenericControlBoard.h"

#include <frc/Joystick.h>

class CowGamepad : public GenericControlBoard
{
private:
    frc::Joystick *m_DriverControlStick;

    bool m_PreviousAuto;

    enum OP_BUTTON_MAP
    {
        CONST_RESET = 5,
        SELECT_AUTO = 6
    };

public:
    CowGamepad();

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

    ~CowGamepad();
};