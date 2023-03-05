#pragma once

#include "RobotCommand.h"
#include "../../CowPigeon.h"
#include "../../CowLib/CowTimer.h"
#include <frc/geometry/Pose2d.h>

class BalanceCommand : public RobotCommand
{
public:
    BalanceCommand(double speed, double timeout, double maxDistance);
    ~BalanceCommand() = default;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

private:
    std::unique_ptr<CowLib::CowTimer> m_Timer;
    CowPigeon &m_Gyro;

    const double m_Speed;
    const double m_Timeout;
    const double m_MaxDistance;

    frc::Pose2d m_StartingPose;

    bool m_PitchHasChanged;

    bool m_Done;

    double m_MaxPitch;
};