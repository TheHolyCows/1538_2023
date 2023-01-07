//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_CONTROL_BOARD_H__
#define __COW_CONTROL_BOARD_H__

#include <frc/Joystick.h>

#define SHIFTER_BUTTON 4
#define AUTON_BUTTON 1
#define PID_BUTTON 3

#define LEFT_GAMEPAD_X 1
#define LEFT_GAMEPAD_Y 2
#define RIGHT_GAMEPAD_X 3
#define RIGHT_GAMEPAD_Y 4
#define STEERING_X 1

#define FAST_TURN 6

// This class offers access to the 2010 specific Cow Control Board
class CowControlBoard
{
private:
    frc::Joystick *m_DriveStick;
    frc::Joystick *m_DriveWheel;
    frc::Joystick *m_OperatorPanel;
    frc::Joystick *m_OperatorGamepad;

    bool m_PreviousAuto;
    //    bool m_PreviousAddAngle;
    //    bool m_PreviousDecAngle;

public:
    CowControlBoard();

    bool GetQuickTurn();
    bool GetAutoSelectButton();
    bool GetAutoAddAngleOffsetButton();
    bool GetAutoDecAngleOffsetButton();

    float GetDriveAxis(unsigned int axis);

    float GetDriveStickY();
    float GetSteeringX();

    float GetOperatorGamepadAxis(unsigned int axis);

    bool GetDriveButton(int button);
    bool GetSteeringButton(int button);
    bool GetOperatorButton(int button);
};

#endif
