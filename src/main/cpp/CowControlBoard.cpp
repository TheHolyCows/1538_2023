#include "CowControlBoard.h"

// Constructor for Cow Control Board
CowControlBoard::CowControlBoard()
    : m_DriveStick(new frc::Joystick(0)),
      m_DriveWheel(new frc::Joystick(1)),
      m_OperatorPanel(new frc::Joystick(2)),
      m_OperatorGamepad(new frc::Joystick(3)),
      m_PreviousAuto(false)
{
}

// Returns state of shifter switch
float CowControlBoard::GetDriveAxis(unsigned int axis)
{
    return m_DriveStick->GetRawAxis(axis);
}

// Returns state of autonomous select button
bool CowControlBoard::GetAutoSelectButton()
{
    if (GetSteeringButton(7) && !m_PreviousAuto)
    {
        m_PreviousAuto = GetSteeringButton(7);
        return true;
    }
    m_PreviousAuto = GetSteeringButton(7);
    return false;
}

// bool CowControlBoard::GetAutoAddAngleOffsetButton()
//{
//     if(GetSteeringButton(9) && !m_PreviousAddAngle)
//     {
//         m_PreviousAddAngle = GetSteeringButton(9);
//         return true;
//     }
//     m_PreviousAddAngle = GetSteeringButton(9);
//     return false;
// }
//
// bool CowControlBoard::GetAutoDecAngleOffsetButton()
//{
//     if(GetSteeringButton(11) && !m_PreviousDecAngle)
//     {
//         m_PreviousDecAngle = GetSteeringButton(11);
//         return true;
//     }
//     m_PreviousDecAngle = GetSteeringButton(11);
//     return false;
// }

// Returns the state of the quick turn
bool CowControlBoard::GetQuickTurn()
{
    return !GetDriveButton(8);
}

float CowControlBoard::GetDriveStickY()
{
    return m_DriveStick->GetRawAxis(1);
}

float CowControlBoard::GetSteeringX()
{
    return m_DriveWheel->GetRawAxis(0);
}

bool CowControlBoard::GetDriveButton(const int button)
{
    return m_DriveStick->GetRawButton(button);
}

bool CowControlBoard::GetOperatorButton(const int button)
{
    return m_OperatorPanel->GetRawButton(button);
}

bool CowControlBoard::GetSteeringButton(const int button)
{
    return m_DriveWheel->GetRawButton(button);
}

float CowControlBoard::GetOperatorGamepadAxis(unsigned int axis)
{
    return m_OperatorGamepad->GetRawAxis(axis);
}
