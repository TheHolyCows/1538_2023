#pragma once

#include "../../CowRobot.h"
#include "RobotCommand.h"

class AprilTagAlignCommand : public RobotCommand
{
private:
    Vision::GamePiece m_GamePiece;

public:
    AprilTagAlignCommand(Vision::GamePiece gamePiece);
    ~AprilTagAlignCommand() = default;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};