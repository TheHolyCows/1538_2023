#include "WaitCommand.h"

WaitCommand::WaitCommand(double timeToWait, bool doNothing)
{
    m_Timer = new CowLib::CowTimer();

    m_DoNothing = doNothing;

    m_TimeToWait = timeToWait;
}

WaitCommand::~WaitCommand()
{
    delete m_Timer;
}

bool WaitCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TimeToWait);
}

void WaitCommand::Start(CowRobot* robot)
{
    m_Timer->Reset();
    m_Timer->Start();
}

void WaitCommand::Handle(CowRobot* robot)
{
    if (m_DoNothing) {
        robot->DoNothing();
    }
}

void WaitCommand::Finish(CowRobot* robot)
{
    m_Timer->Stop();
}
