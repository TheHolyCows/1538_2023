#include "RaceCommand.h"

RaceCommand::RaceCommand(RobotCommand* leadCommand, std::vector<RobotCommand*> otherCommands)
{
    m_LeadCommand = leadCommand;
    m_OtherCommands = otherCommands;
}

RaceCommand::~RaceCommand()
{
    // TODO: figure out if these need to be deleted here (don't think so)

    // delete m_LeadCommand;
    // for (RobotCommand* command : m_OtherCommands)
    // {
    //     delete command;
    // }
}

bool RaceCommand::IsComplete(CowRobot *robot)
{
    return m_LeadCommand->IsComplete(robot);
}

void RaceCommand::Start(CowRobot* robot)
{
    m_LeadCommand->Start(robot);
    for (RobotCommand* command : m_OtherCommands) {
        command->Start(robot);
    }
}

void RaceCommand::Handle(CowRobot* robot)
{
    m_LeadCommand->Handle(robot);
    for (RobotCommand* command : m_OtherCommands) {
        command->Handle(robot);
    }
}

void RaceCommand::Finish(CowRobot* robot)
{
    m_LeadCommand->Finish(robot);
    for (RobotCommand* command : m_OtherCommands) {
        command->Finish(robot);
    }
}