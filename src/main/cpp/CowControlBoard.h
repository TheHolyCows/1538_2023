//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_CONTROL_BOARD_H__
#define __COW_CONTROL_BOARD_H__

#include <frc/Joystick.h>

// This class offers access to the 2010 specific Cow Control Board
// LOL this is 2023 now
class CowControlBoard
{
private:
    frc::Joystick *m_LeftDriveStick;
    frc::Joystick *m_RightDriveStick;
    frc::Joystick *m_OperatorPanel;

    bool m_PreviousAuto;

public:
    CowControlBoard();

    bool GetAutoSelectButton();

    double GetLeftDriveStickAxis(unsigned int axis);
    double GetRightDriveStickAxis(unsigned int axis);

    double GetLeftDriveStickY();
    double GetRightDriveStickY();

    bool GetLeftDriveStickButton(const int button);
    bool GetRightDriveStickButton(const int button);

    bool GetOperatorButton(const int button);
};

#endif
