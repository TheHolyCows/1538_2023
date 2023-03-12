#include "PathplannerSwerveTrajectoryCommand.h"

PathplannerSwerveTrajectoryCommand::PathplannerSwerveTrajectoryCommand(const std::string &trajectoryName,
                                                                       double maxSpeed,
                                                                       double maxAccel,
                                                                       bool stopAtEnd,
                                                                       bool resetOdometry,
                                                                       std::vector<Event> events)
{
    // This is to make sure that it is loading trajectories on start and not on demand
    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
    m_ResetOdometry = resetOdometry;

    // Load trajectory from file
    m_Trajectory = pathplanner::PathPlanner::loadPath(trajectoryName,
                                                      units::feet_per_second_t{ maxSpeed },
                                                      units::feet_per_second_squared_t{ maxAccel });

    m_Trajectory
        = pathplanner::PathPlannerTrajectory::transformTrajectoryForAlliance(m_Trajectory,
                                                                             frc::DriverStation::GetAlliance());

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Loaded trajectory %s", trajectoryName.c_str());

    m_HolonomicController = new pathplanner::PPHolonomicDriveController(
        frc2::PIDController{ CONSTANT("AUTO_DRIVE_P"), CONSTANT("AUTO_DRIVE_I"), CONSTANT("AUTO_DRIVE_D") },
        frc2::PIDController{ CONSTANT("AUTO_DRIVE_P"), CONSTANT("AUTO_DRIVE_I"), CONSTANT("AUTO_DRIVE_D") },
        frc2::PIDController{ CONSTANT("AUTO_ROTATION_P"), CONSTANT("AUTO_ROTATION_I"), CONSTANT("AUTO_ROTATION_D") });

    m_TotalTime = m_Trajectory.getTotalTime().value();

    m_Events = events;

    std::vector<pathplanner::PathPlannerTrajectory::EventMarker> markers = m_Trajectory.getMarkers();

    for (Event event : m_Events)
    {
        for (pathplanner::PathPlannerTrajectory::EventMarker marker : markers)
        {
            if (std::find(marker.names.begin(), marker.names.end(), event.waypointName) != marker.names.end())
            {
                event.time = marker.time.value();
            }
        }
    }
}

PathplannerSwerveTrajectoryCommand::~PathplannerSwerveTrajectoryCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool PathplannerSwerveTrajectoryCommand::IsComplete(CowRobot *robot)
{
    return m_Timer->HasElapsed(m_TotalTime);
}

void PathplannerSwerveTrajectoryCommand::Start(CowRobot *robot)
{
    if (m_ResetOdometry)
    {
        robot->GetDrivetrain()->ResetOdometry(m_Trajectory.getInitialHolonomicPose());

        // auto initPose = m_Trajectory.InitialPose();
        // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
        //                           "Initial pose: x %f, y %f, d %f",
        //                           initPose.X().convert<units::foot>().value(),
        //                           initPose.Y().convert<units::foot>().value(),
        //                           initPose.Rotation().Degrees().value());
    }

    m_Timer->Reset();
    m_Timer->Start();
}

void PathplannerSwerveTrajectoryCommand::Handle(CowRobot *robot)
{
    for (Event event : m_Events)
    {
        if (m_Timer->HasPeriodPassed(event.time) && !event.done)
        {
            if (!event.started)
            {
                event.command->Start(robot);
                event.started = true;
            }

            if (event.command->IsComplete(robot))
            {
                event.done = true;
                event.command->Finish(robot);
            }
            else
            {
                event.command->Handle(robot);
            }
        }
    }

    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    pathplanner::PathPlannerTrajectory::PathPlannerState targetState
        = m_Trajectory.sample(units::second_t{ m_Timer->Get() });

    CowLib::CowChassisSpeeds chassisSpeeds
        = CowLib::CowChassisSpeeds::FromWPI(m_HolonomicController->calculate(currentPose, targetState));
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "vx: %f, vy: %f, o: %f",
    //                           chassisSpeeds.vx,
    //                           chassisSpeeds.vy,
    //                           chassisSpeeds.omega);
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "x %f y %f angle %f",
    //                           currentPose.X().value(),
    //                           currentPose.Y().value(),
    //                           currentPose.Rotation().Degrees().value());

    // auto err = frc::Transform2d(currentPose, targetState.pose);
    // frc::SmartDashboard::PutNumber("auto/swerve/error/x", err.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/error/y", err.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/error/rotation", err.Rotation().Degrees().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/x",
    //                                currentPose.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/y",
    //                                currentPose.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/actual/rotation", currentPose.Rotation().Degrees().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/x",
    //                                targetState.asWPILibState().pose.Translation().X().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/y",
    //                                targetState.asWPILibState().pose.Translation().Y().convert<units::foot>().value());
    // frc::SmartDashboard::PutNumber("auto/swerve/target/rotation",
    //                                targetState.asWPILibState().pose.Rotation().Degrees().value());

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void PathplannerSwerveTrajectoryCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Stopping swerve trajectory command");
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    for (Event event : m_Events)
    {
        if (!event.done && event.started)
        {
            event.command->Finish(robot);
        }
    }

    m_Timer->Stop();
}

frc::Pose2d PathplannerSwerveTrajectoryCommand::GetStartingPose()
{
    return m_Trajectory.getInitialHolonomicPose();
}
