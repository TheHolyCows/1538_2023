#ifndef __SWERVE_TRAJECTORY_COMMAND_H__
#define __SWERVE_TRAJECTORY_COMMAND_H__

#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

#include <iostream>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <pathplanner/lib/PathPlanner.h>
#include <string>

class PathplannerSwerveTrajectoryCommand : public RobotCommand
{
private:
    CowLib::CowTimer *m_Timer;
    pathplanner::PathPlannerTrajectory m_Trajectory;
    pathplanner::PPHolonomicDriveController *m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    bool m_ResetOdometry;

public:
    PathplannerSwerveTrajectoryCommand(const std::string &trajectoryName,
                                       double maxSpeed,
                                       double maxAccel,
                                       bool stop,
                                       bool resetOdometry = false);
    PathplannerSwerveTrajectoryCommand(pathplanner::PathPlannerTrajectory &trajectory,
                                       bool stop,
                                       bool resetOdometry = false);
    ~PathplannerSwerveTrajectoryCommand() override;

    bool IsComplete() override;

    void Start(CowRobot *robot) override;

    void Handle(CowRobot *robot) override;

    void Finish(CowRobot *robot) override;

    frc::Pose2d GetStartingPose();
};

#endif /* __SWERVE_TRAJECTORY_COMMAND_H__ */
