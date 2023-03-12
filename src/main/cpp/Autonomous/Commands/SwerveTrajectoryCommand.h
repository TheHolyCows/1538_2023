#ifndef __SWERVE_TRAJECTORY_COMMAND_H__
#define __SWERVE_TRAJECTORY_COMMAND_H__

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

class SwerveTrajectoryCommand : public RobotCommand
{
private:
    CowLib::CowTimer *m_Timer;
    frc::Trajectory m_Trajectory;
    CowLib::CowHolonomicController *m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    double m_TargetAngle;
    bool m_ResetOdometry;

public:
    SwerveTrajectoryCommand(const std::string &trajectoryName,
                            double targetAngle,
                            bool stop,
                            bool resetOdometry = false);
    ~SwerveTrajectoryCommand() override;

    bool IsComplete(CowRobot *robot) override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

    frc::Pose2d GetStartingPose();
};

#endif /* __SWERVE_TRAJECTORY_COMMAND_H__ */
