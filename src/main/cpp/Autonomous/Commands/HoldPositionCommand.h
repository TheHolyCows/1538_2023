#ifndef __HOLD_POSITION_COMMAND_H__
#define __HOLD_POSITION_COMMAND_H__

#include "../../CowLib/CowHolonomicController.h"
#include "../../CowLib/CowLatch.h"
#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <iostream>
#include <string>
#include <wpi/fs.h>

class HoldPositionCommand : public RobotCommand
{
private:
    CowLib::CowTimer *m_Timer;
    CowLib::CowHolonomicController *m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;
    double m_MaxVelocity;

    frc::Pose2d m_Pose;

public:
    HoldPositionCommand(frc::Pose2d pose, double time, double maxVelocity, bool stopAtEnd, bool resetOdometry = false);
    ~HoldPositionCommand() override;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};

#endif /* __HOLD_POSITION_COMMAND_H__ */
