#include "CommandRunner.h"

CommandRunner::CommandRunner(RobotCommand &command, double timeout)
    : m_Command(command),
      m_State(NOT_STARTED),
      m_Timeout({ timeout > 0, timeout }),
      m_Timer(new CowLib::CowTimer())
{
}

void CommandRunner::Start(CowRobot *robot)
{
    m_State = RUNNING;
    m_Timer->Start();
    m_Command.Start(robot);
}

void CommandRunner::Handle(CowRobot *robot)
{
    if (m_State != RUNNING)
    {
        return;
    }

    if (m_Timeout.ShouldStop(m_Timer->Get()))
    {
        m_State = FINISHED;
        return;
    }

    m_Command.Handle(robot);
}

void CommandRunner::Finish(CowRobot *robot)
{
    m_Command.Finish(robot);
    m_State = FINISHED;
    m_Timer->Stop();
}