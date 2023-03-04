#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <memory>

class CubeAlignCommand : public RobotCommand
{
private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;

    const double m_Timeout;

public:
    CubeAlignCommand(const double timeout);
    ~CubeAlignCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};