#include "LambdaCommand.h"

LambdaCommand::LambdaCommand(std::function<void(CowRobot *)> lambda)
{
    m_Lambda = lambda;
}

LambdaCommand::~LambdaCommand() = default;

bool LambdaCommand::IsComplete(CowRobot *robot)
{
    return true;
}

void LambdaCommand::Start(CowRobot *robot)
{
    m_Lambda(robot);
}

void LambdaCommand::Handle(CowRobot *robot)
{
}

void LambdaCommand::Finish(CowRobot *robot)
{
}
