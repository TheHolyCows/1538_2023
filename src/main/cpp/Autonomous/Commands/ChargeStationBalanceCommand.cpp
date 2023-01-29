#include "ChargeStationBalanceCommand.h"

ChargeStationBalanceCommand::ChargeStationBalanceCommand(double timeout)
{
    m_Timeout = timeout;

    m_Timer = new CowLib::CowTimer();
}

ChargeStationBalanceCommand::~ChargeStationBalanceCommand()
{
}

bool ChargeStationBalanceCommand::IsComplete()
{
    return true;
}

void ChargeStationBalanceCommand::Start(CowRobot *robot)
{
    m_Timer->Start();
}

void ChargeStationBalanceCommand::Handle(CowRobot *robot)
{
}

void ChargeStationBalanceCommand::Finish(CowRobot *robot)
{
}