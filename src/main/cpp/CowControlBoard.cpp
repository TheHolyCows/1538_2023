#include "CowControlBoard.h"

// Constructor for Cow Control Board
CowControlBoard::CowControlBoard()
    : m_LeftDriveStick(new frc::Joystick(0)),
      m_RightDriveStick(new frc::Joystick(1)),
      m_OperatorPanel(new frc::Joystick(2)),
      m_PreviousAuto(false)
{
}

// Returns state of autonomous select button
bool CowControlBoard::GetAutoSelectButton()
{
    // TODO: change this
    if (GetOperatorButton(7) && !m_PreviousAuto)
    {
        m_PreviousAuto = GetOperatorButton(7);
        return true;
    }
    m_PreviousAuto = GetOperatorButton(7);
    return false;
}

double CowControlBoard::GetLeftDriveStickAxis(unsigned int axis)
{
    return m_LeftDriveStick->GetRawAxis(axis);
}

double CowControlBoard::GetRightDriveStickAxis(unsigned int axis)
{
    return m_RightDriveStick->GetRawAxis(axis);
}

double CowControlBoard::GetLeftDriveStickY()
{
    return m_LeftDriveStick->GetRawAxis(1);
}

double CowControlBoard::GetRightDriveStickY()
{
    return m_RightDriveStick->GetRawAxis(1);
}

bool CowControlBoard::GetLeftDriveStickButton(const int button)
{
    return m_LeftDriveStick->GetRawButton(button);
}

bool CowControlBoard::GetRightDriveStickButton(const int button)
{
    return m_LeftDriveStick->GetRawButton(button);
}

bool CowControlBoard::GetOperatorButton(const int button)
{
    return m_OperatorPanel->GetRawButton(button);
}
