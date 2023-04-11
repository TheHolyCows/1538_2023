#include "SeriesCommand.h"

SeriesCommand::SeriesCommand(std::deque<RobotCommand*> commands)
{
    m_Commands = commands;
}

SeriesCommand::~SeriesCommand()
{
    // TODO: figure out if they need to be deleted here (don't think so)
}

bool SeriesCommand::IsComplete(CowRobot *robot)
{
    return m_Commands.empty() && m_CurrentCommand == nullptr;
}

void SeriesCommand::Start(CowRobot* robot)
{
    m_CurrentCommand = m_Commands.front();
    m_Commands.pop_front();
    m_CurrentCommand->Start(robot);
}

void SeriesCommand::Handle(CowRobot* robot)
{
    if (m_CurrentCommand == nullptr) {
        // This should never happen
        return;
    }

    if (m_CurrentCommand->IsComplete(robot)) {
        m_CurrentCommand->Finish(robot);

        if (m_Commands.empty()) {
            m_CurrentCommand = nullptr;
            return;
        }
        else {
            m_CurrentCommand = m_Commands.front();
            m_Commands.pop_front();
            m_CurrentCommand->Start(robot);
        }
    }

    m_CurrentCommand->Handle(robot);
}

void SeriesCommand::Finish(CowRobot* robot) {}