#include "AutoModes.h"

#include <frc/Errors.h>

AutoModes *AutoModes::m_SingletonInstance = NULL;

AutoModes *AutoModes::GetInstance()
{
    if (m_SingletonInstance == NULL)
    {
        m_SingletonInstance = new AutoModes();
    }
    return m_SingletonInstance;
}

AutoModes::AutoModes()
{
    /** Do Nothing **/
    m_Modes["Nothing"];
    m_Modes["Nothing"].push_back(RobotCommand(CMD_WAIT, 0, 0, 0, 1));

    /** Move Around **/
    m_Modes["Leave Start Zone"];
    m_Modes["Leave Start Zone"].push_back(RobotCommand(CMD_HOLD_DISTANCE, -55, 0, 0.4, 4));

    m_Modes["Leave And Ret"];
    m_Modes["Leave And Ret"].push_back(RobotCommand(CMD_HOLD_DISTANCE, -55, 0, 0.4, 4));
    m_Modes["Leave And Ret"].push_back(RobotCommand(CMD_HOLD_DISTANCE, 0, 0, 0.4, 4));

    m_Modes["Test_Turn"];
    m_Modes["Test_Turn"].push_back(RobotCommand(CMD_WAIT, 0, 0, 0, 1));
    m_Modes["Test_Turn"].push_back(RobotCommand(CMD_TURN, 0, 45, 0.2, 1));
    m_Modes["Test_Turn"].push_back(RobotCommand(CMD_WAIT, 0, 0, 0, 1));

    m_Iterator = m_Modes.begin();
}

std::deque<RobotCommand> AutoModes::GetCommandList()
{
    return m_Iterator->second;
}

const char *AutoModes::GetName()
{
    return m_Iterator->first;
}

void AutoModes::NextMode()
{
    ++m_Iterator;
    if (m_Iterator == m_Modes.end())
    {
        m_Iterator = m_Modes.begin();
    }
    std::string str(GetName());
    std::string temp = "Auto mode: " + str;
    FRC_ReportError(frc::err::Error, "{}", temp);
}