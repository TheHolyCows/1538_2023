#include "ParallelCommand.h"

ParallelCommand::ParallelCommand(std::vector<RobotCommand*> commands)
{
    m_Commands = commands;
}

ParallelCommand::~ParallelCommand()
{
    // TODO: figure out if these need to be deleted here (don't think so)

    // delete m_LeadCommand;
    // for (RobotCommand* command : m_OtherCommands)
    // {
    //     delete command;
    // }
}

bool ParallelCommand::IsComplete(CowRobot *robot)
{
    for (RobotCommand* command : m_Commands) {
        if (!command->IsComplete(robot)) {
            return false;
        }
    }

    return true;
}

void ParallelCommand::Start(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Start(robot);
    }
}

void ParallelCommand::Handle(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Handle(robot);
    }
}

void ParallelCommand::Finish(CowRobot* robot)
{
    for (RobotCommand* command : m_Commands) {
        command->Finish(robot);
    }
}