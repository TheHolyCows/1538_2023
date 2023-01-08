#ifndef __SWERVE_TRAJECTORY_COMMAND_H__
#define __SWERVE_TRAJECTORY_COMMAND_H__

#include <iostream>
#include <string>

#include <frc/Filesystem.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

#include "../../CowLib/CowHolonomicController.h"
#include "../../CowLib/CowTimer.h"
#include "../../CowRobot.h"
#include "./RobotCommand.h"

class SwerveTrajectoryCommand : public RobotCommand {
private:
    CowLib::CowTimer* m_Timer;
    frc::Trajectory m_Trajectory;
    CowLib::CowHolonomicController* m_HolonomicController;

    double m_TotalTime;
    bool m_Stop;
    double m_TargetAngle;

public:
    SwerveTrajectoryCommand(const std::string& trajectoryName, double targetAngle, bool stop);
    ~SwerveTrajectoryCommand() override;

    bool IsComplete() override;

    void Start(CowRobot* robot) override;

    void Handle(CowRobot* robot) override;

    void Finish(CowRobot* robot) override;
};

#endif /* __SWERVE_TRAJECTORY_COMMAND_H__ */
