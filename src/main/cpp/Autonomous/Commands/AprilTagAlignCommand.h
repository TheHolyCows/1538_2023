#pragma once

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "RobotCommand.h"

#include <memory>

class AprilTagAlignCommand : public RobotCommand
{
private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;

    Vision::GamePiece m_GamePiece;
    const double m_Timeout;

public:
    AprilTagAlignCommand(Vision::GamePiece gamePiece, const double timeout);
    ~AprilTagAlignCommand() = default;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};