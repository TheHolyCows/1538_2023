//==================================================
// Copyright (C) 2023 Team 1538 / The Holy Cows
// GenericControlBoard.h
// author: jon-bassi
//==================================================

#pragma once /* __GENERIC_CONTROL_BOARD_H__ */

class GenericControlBoard
{
public:
    virtual bool GetAutoSelectButton()     = 0;
    virtual bool GetConstantsResetButton() = 0;
    virtual bool GetRobotRelativeButton()  = 0;
    virtual bool GetVisionTargetButton()   = 0;

    virtual bool GetDriveButton(int) = 0;
    virtual double GetDriveAxis(int) = 0;

    virtual double GetLeftDriveStickX() = 0;
    virtual double GetLeftDriveStickY() = 0;

    virtual double GetRightDriveStickX() = 0;
    virtual double GetRightDriveStickY() = 0;

    virtual bool GetOperatorButton(int) = 0;
    virtual double GetOperatorAxis(int) = 0;

    virtual ~GenericControlBoard(){};
};