#ifndef __CHARGE_STATION_BALANCE_COMMAND_H__
#define __CHARGE_STATION_BALANCE_COMMAND_H__

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

class ChargeStationBalanceCommand : public RobotCommand
{
private:
    double m_Timeout;

    CowLib::CowTimer *m_Timer;

public:
    ChargeStationBalanceCommand(double timeout);
    ~ChargeStationBalanceCommand() override;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};

#endif /* __CHARGE_STATION_BALANCE_COMMAND_H__ */
