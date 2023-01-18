#include "CowDisplay.h"

#include <frc/RobotController.h>

CowDisplay::CowDisplay(CowRobot *bot)
    : m_UserState(0),
      m_PrevUserState(0),
      m_UserStatePeriodicCount(0),
      m_UserPeriodicCount(0),
      m_UserScrollCount(0),
      m_ButtonPressedOnce(false),
      m_Bot(bot)
{
    m_Bot->GetDisplay()->SetBanner("Team 1538 The Holy Cows ");
}

void CowDisplay::DisplayUpdateState()
{
    CowLib::CowAlphaNum *alphaNumLED = m_Bot->GetDisplay();
    switch (m_UserState)
    {
    case 0 :
        if (m_PrevUserState != m_UserState)
        {
            m_PrevUserState   = m_UserState;
            m_UserScrollCount = 0;
            alphaNumLED->SetBanner("Team 1538 The Holy Cows ");
        }
        else
        {
            m_UserScrollCount++;
        }
        alphaNumLED->SetBannerPosition(m_UserScrollCount);
        alphaNumLED->DisplayBanner();
        break;
    case 1 :
        m_UserScrollCount = 0; // Not scrolling the voltage
        if (m_PrevUserState != m_UserState)
        {
            m_PrevUserState = m_UserState;
            alphaNumLED->SetBanner("Volt");
        }
        else
        {
            if (m_UserStatePeriodicCount == 50)
            {
                char volt[64];
                // TODO: FIX
                sprintf(volt, "%.2f ", -65535.0f); // frc::DriverStation::GetBatteryVoltage();
                std::string msg(volt);
                msg = msg + " ";
                alphaNumLED->SetBanner(msg);
            }
        }
        alphaNumLED->SetBannerPosition(m_UserScrollCount);
        alphaNumLED->DisplayBanner();
        break;
    case 2 :
        m_UserScrollCount = 0; // not scrolling the voltage
        if (m_PrevUserState != m_UserState)
        {
            m_PrevUserState = m_UserState;
            alphaNumLED->SetBanner("Gyro");
        }
        else
        {
            if (m_UserStatePeriodicCount == 50)
            {
                char volt[64];
                // sprintf(volt, "%.2f ", m_Bot->GetGyro()->GetRate());
                sprintf(volt, "%.2f ", 0.0); // Pigeon has no rate...
                std::string msg(volt);
                msg = msg + " ";
                alphaNumLED->SetBanner(msg);
            }
        }
        alphaNumLED->SetBannerPosition(m_UserScrollCount);
        alphaNumLED->DisplayBanner();
        break;
    default :
        break;
    }
}

void CowDisplay::DisplayNextState()
{
    switch (m_UserState)
    {
    case 0 :
        m_PrevUserState = 0;
        m_UserState     = 1;
        break;
    case 1 :
        m_PrevUserState = 1;
        m_UserState     = 2;
        break;
    case 2 :
        m_PrevUserState = 2;
        m_UserState     = 0;
        break;
    default :
        break;
    }
}

void CowDisplay::DisplayState(bool user)
{
    if (user)
    {
        m_UserStatePeriodicCount = 0;
        DisplayNextState();
        DisplayUpdateState();
    }
    else
    {
        if ((m_UserState == 0) && ((m_UserStatePeriodicCount % 10) == 0))
        {
            DisplayUpdateState();
        }
        else if ((m_UserStatePeriodicCount % 300) == 0)
        {
            m_UserStatePeriodicCount = 0;
            DisplayNextState();
            DisplayUpdateState();
        }
        else if ((m_UserStatePeriodicCount % 10) == 0)
        {
            DisplayUpdateState();
        }
    }
}

void CowDisplay::DisplayPeriodic()
{
    bool userButtonPressed = frc::RobotController::GetUserButton();
    bool buttonValue       = false;

    if (userButtonPressed && !m_ButtonPressedOnce)
    {
        buttonValue         = true;
        m_ButtonPressedOnce = true;
    }
    else if (!userButtonPressed && m_ButtonPressedOnce)
    {
        m_ButtonPressedOnce = false;
    }

    m_UserPeriodicCount++;
    m_UserStatePeriodicCount++;

    DisplayState(buttonValue);
}
