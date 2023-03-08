#include "NullCommand.h"

bool NullCommand::IsComplete(CowRobot *robot)
{
    return true;
}

void NullCommand::Start(CowRobot* robot)
{
}

void NullCommand::Handle(CowRobot* robot)
{
}

void NullCommand::Finish(CowRobot* robot)
{
}