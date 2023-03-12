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

    frc2::PIDController *m_XController;
    frc2::PIDController *m_YController;
    frc2::PIDController *m_RotationController;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;
    double m_MaxVelocity;

    frc::Pose2d m_Pose;

public:
    HoldPositionCommand(double time, double maxVelocity, bool stopAtEnd, bool resetOdometry = false);
    ~HoldPositionCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;
};

#endif /* __HOLD_POSITION_COMMAND_H__ */
