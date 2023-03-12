#include "ClawCommand.h"

ClawCommand::ClawCommand(CLAW_STATE state, double timeout)
{
    m_Timer = std::make_unique<CowLib::CowTimer>();

    m_ClawState = state;
    m_Timeout   = timeout;
    m_Wait      = timeout > 0;
}

bool ClawCommand::IsComplete(CowRobot *robot)
{
    if (!m_Wait)
    {
        return true;
    }

    return m_Timer->HasElapsed(m_Timeout);
}

void ClawCommand::Start(CowRobot *robot)
{
    m_Timer->Reset();
    m_Timer->Start();
    robot->GetArm()->SetClawState(m_ClawState);
    robot->GetArm()->UpdateClawState();
}

void ClawCommand::Handle(CowRobot *robot)
{
    robot->GetArm()->UpdateClawState();
    return;
}

void ClawCommand::Finish(CowRobot *robot)
{
    if (m_Wait)
    {
        robot->GetArm()->SetClawState(CLAW_STATE::CLAW_OFF);
    }

    m_Timer->Stop();

    return;
}