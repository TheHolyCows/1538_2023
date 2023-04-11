#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

#include <iostream>
#include <memory>

class MaintainVelocityCommand : public RobotCommand
{
public:
    MaintainVelocityCommand(double targetVelocity,
                            double timeout,
                            bool maintainHeading,
                            bool stop          = false,
                            bool fieldOriented = true);
    ~MaintainVelocityCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

private:
    const double m_TargetVelocity;
    const double m_Timeout;
    const bool m_MaintainHeading;
    const bool m_FieldOriented;
    const bool m_Stop;

    std::unique_ptr<CowLib::CowTimer> m_Timer;
};
