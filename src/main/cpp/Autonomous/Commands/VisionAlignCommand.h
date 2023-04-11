#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"
#include "../../CowPigeon.h"

#include <memory>

class VisionAlignCommand : public RobotCommand
{
private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;

    CowPigeon& m_Gyro;

    const double m_Timeout;

    ARM_CARGO m_Cargo;

public:
    VisionAlignCommand(double timeout, ARM_CARGO cargo);
    ~VisionAlignCommand() override = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};