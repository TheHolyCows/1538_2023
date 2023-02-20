#include "CowGamepad.h"

// Constructor for Cow Control Board
CowGamepad::CowGamepad()
    : m_DriverControlStick(new frc::Joystick(0)),
      m_PreviousAuto(false)
{
}

// Returns state of autonomous select button
bool CowGamepad::GetAutoSelectButton()
{
    // if (GetOperatorButton(CONST_RESET) && !m_PreviousAuto)
    // {
    //     m_PreviousAuto = GetOperatorButton(CONST_RESET);
    //     return true;
    // }
    // m_PreviousAuto = GetOperatorButton(CONST_RESET);
    return false;
}

bool CowGamepad::GetConstantsResetButton()
{
    // TODO: change this
    // if (GetOperatorButton(SELECT_AUTO) && !m_PreviousAuto)
    // {
    //     m_PreviousAuto = GetOperatorButton(SELECT_AUTO);
    //     return true;
    // }
    // m_PreviousAuto = GetOperatorButton(SELECT_AUTO);
    return false;
}

bool CowGamepad::GetRobotRelativeButton()
{
    return (GetDriveAxis(2) > 0.85);
}

bool CowGamepad::GetVisionTargetButton()
{
    return (GetDriveAxis(3) > 0.85);
}

bool CowGamepad::GetDriveButton(int button)
{
    return m_DriverControlStick->GetRawButton(button);
}

double CowGamepad::GetDriveAxis(int axis)
{
    return m_DriverControlStick->GetRawAxis(axis);
}

double CowGamepad::GetLeftDriveStickX()
{
    return m_DriverControlStick->GetRawAxis(0) * -1;
}

double CowGamepad::GetLeftDriveStickY()
{
    return m_DriverControlStick->GetRawAxis(1) * -1;
}

double CowGamepad::GetRightDriveStickX()
{
    return m_DriverControlStick->GetRawAxis(4) * -1;
}

double CowGamepad::GetRightDriveStickY()
{
    return m_DriverControlStick->GetRawAxis(5);
}

bool CowGamepad::GetOperatorButton(int button)
{
    return m_DriverControlStick->GetRawButton(button);
}

double CowGamepad::GetOperatorAxis(int axis)
{
    return m_DriverControlStick->GetRawAxis(axis);
}

CowGamepad::~CowGamepad()
{
    delete m_DriverControlStick;
}