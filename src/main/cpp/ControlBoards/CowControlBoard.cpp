#include "CowControlBoard.h"

// Constructor for Cow Control Board
CowControlBoard::CowControlBoard()
    : m_DriverControlStick(new frc::Joystick(0)),
      m_OperatorPanel(new frc::Joystick(1)),
      m_OperatorControlStick(new frc::Joystick(2)),
      m_PreviousAuto(false),
      m_PreviousReset(false)
{
}

// Returns state of autonomous select button
bool CowControlBoard::GetAutoSelectButton()
{
    // TODO: change this
    if (GetOperatorButton(BT_SELECT_AUTO) && !m_PreviousAuto)
    {
        m_PreviousAuto = GetOperatorButton(BT_SELECT_AUTO);
        return true;
    }
    m_PreviousAuto = GetOperatorButton(BT_SELECT_AUTO);
    return false;
}

bool CowControlBoard::GetConstantsResetButton()
{
    if (GetOperatorButton(BT_CONST_RESET) && !m_PreviousReset)
    {
        m_PreviousReset = GetOperatorButton(BT_CONST_RESET);
        return true;
    }
    m_PreviousReset = GetOperatorButton(BT_CONST_RESET);
    return false;
}

bool CowControlBoard::GetRobotRelativeButton()
{
    return (GetDriveAxis(2) > 0.85);
}

bool CowControlBoard::GetVisionTargetButton()
{
    return (GetDriveAxis(5) > 0.85);
}

bool CowControlBoard::GetDriveButton(int button)
{
    return m_DriverControlStick->GetRawButton(button);
}

double CowControlBoard::GetDriveAxis(int axis)
{
    return m_DriverControlStick->GetRawAxis(axis);
}

double CowControlBoard::GetLeftDriveStickX()
{
    return m_DriverControlStick->GetRawAxis(0) * -1;
}

double CowControlBoard::GetLeftDriveStickY()
{
    return m_DriverControlStick->GetRawAxis(1) * -1;
}

double CowControlBoard::GetRightDriveStickX()
{
    return m_DriverControlStick->GetRawAxis(4);
}

double CowControlBoard::GetRightDriveStickY()
{
    return m_DriverControlStick->GetRawAxis(5);
}

bool CowControlBoard::GetOperatorButton(int button)
{
    return m_OperatorPanel->GetRawButton(button);
}

double CowControlBoard::GetOperatorAxis(int axis)
{
    return m_OperatorControlStick->GetRawAxis(axis);
}

CowControlBoard::~CowControlBoard()
{
    delete m_DriverControlStick;
    delete m_OperatorPanel;
    delete m_OperatorControlStick;
}