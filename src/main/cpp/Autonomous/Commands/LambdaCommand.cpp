#include "LambdaCommand.h"

LambdaCommand::LambdaCommand(std::function<void(CowRobot*)> lambda)
{
    m_Lambda = lambda;
}

LambdaCommand::~LambdaCommand()
    = default;

bool LambdaCommand::IsComplete()
{
    return true;
}

void LambdaCommand::Start(CowRobot* robot)
{
    std::cout << "LambdaCommand::Start" << std::endl;
    m_Lambda(robot);
}

void LambdaCommand::Handle(CowRobot* robot) {}

void LambdaCommand::Finish(CowRobot* robot) {}
