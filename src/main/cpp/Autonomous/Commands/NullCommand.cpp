#include "NullCommand.h"

bool NullCommand::IsComplete()
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