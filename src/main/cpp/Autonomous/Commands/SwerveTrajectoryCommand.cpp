#include "SwerveTrajectoryCommand.h"

SwerveTrajectoryCommand::SwerveTrajectoryCommand(const std::string &trajectoryName,
                                                 double targetAngle,
                                                 bool stopAtEnd,
                                                 bool resetOdometry)
{
    // This is to make sure that it is loading trajectories on start and not on demand
    std::cout << "Loading swerve trajectory " << trajectoryName << std::endl;

    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
    m_TargetAngle   = targetAngle;
    m_ResetOdometry = resetOdometry;

    // Load trajectory from file
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    fs::path path            = deployDirectory / "trajectories" / (trajectoryName + ".wpilib.json");

    std::cout << "Got path " << path << std::endl;

    m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(path.string());

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Loaded trajectory %s", trajectoryName.c_str());

    m_HolonomicController = new CowLib::CowHolonomicController(CONSTANT("AUTO_DRIVE_P"),
                                                               CONSTANT("AUTO_DRIVE_I"),
                                                               CONSTANT("AUTO_DRIVE_D"),
                                                               CONSTANT("AUTO_ROTATION_P"),
                                                               CONSTANT("AUTO_ROTATION_I"),
                                                               CONSTANT("AUTO_ROTATION_D"),
                                                               CONSTANT("SWERVE_MAX_ANGULAR_VELOCITY"),
                                                               CONSTANT("SWERVE_MAX_ROTATIONAL_ACCEL"));

    m_TotalTime = m_Trajectory.TotalTime().value();

    std::cout << "Completed constructing swerve command for " << trajectoryName << std::endl;
}

SwerveTrajectoryCommand::~SwerveTrajectoryCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool SwerveTrajectoryCommand::IsComplete(CowRobot *robot)
{
    // I think these units are correct but could be problem spot

    // More probably has to be added here

    return m_Timer->HasElapsed(m_TotalTime);
}

void SwerveTrajectoryCommand::Start(CowRobot *robot)
{
    if (m_ResetOdometry)
    {
        robot->GetDrivetrain()->ResetOdometry(m_Trajectory.InitialPose());

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

void SwerveTrajectoryCommand::Handle(CowRobot *robot)
{
    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    frc::Trajectory::State targetPose = m_Trajectory.Sample(units::second_t{ m_Timer->Get() });

    CowLib::CowChassisSpeeds chassisSpeeds = m_HolonomicController->Calculate(currentPose, targetPose, m_TargetAngle);
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

    // Vision align logic has to go here

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void SwerveTrajectoryCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "Stopping swerve trajectory command");
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}

frc::Pose2d SwerveTrajectoryCommand::GetStartingPose()
{
    return m_Trajectory.InitialPose();
}
