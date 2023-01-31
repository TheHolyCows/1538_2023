#ifndef __CHARGE_STATION_BALANCE_COMMAND_H__
#define __CHARGE_STATION_BALANCE_COMMAND_H__

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "frc/controller/PIDController.h"
#include "RobotCommand.h"
#include "units/angle.h"

#include <frc/controller/ProfiledPIDController.h>

class ChargeStationBalanceCommand : public RobotCommand
{
private:
    double m_Timeout;

    CowLib::CowTimer *m_Timer;

    bool m_StartedDrivingUp;

    frc2::PIDController *m_PIDController;

public:
    ChargeStationBalanceCommand(double timeout);
    ~ChargeStationBalanceCommand() override;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};

#endif /* __CHARGE_STATION_BALANCE_COMMAND_H__ */
