#include "SwerveTrajectoryCommand.h"

SwerveTrajectoryCommand::SwerveTrajectoryCommand(const std::string& trajectoryName, double targetAngle, bool stopAtEnd)
{
    // This is to make sure that it is loading trajectories on start and not on demand
    std::cout << "Loading swerve trajectory " << trajectoryName << std::endl;

    m_Timer = new CowLib::CowTimer();
    m_Stop = stopAtEnd;
    m_TargetAngle = targetAngle;

    // Load trajectory from file
    fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
    fs::path path = deployDirectory / "trajectories" / (trajectoryName + ".wpilib.json");

    std::cout << "Got path " << path << std::endl;

    m_Trajectory = frc::TrajectoryUtil::FromPathweaverJson(path.string());

    std::cout << "Loaded trajectory " << trajectoryName << std::endl;

    m_HolonomicController = new CowLib::CowHolonomicController(
        CONSTANT("AUTO_P"), CONSTANT("AUTO_I"), CONSTANT("AUTO_D"),
        CONSTANT("AUTO_ROTATION_P"), CONSTANT("AUTO_ROTATION_I"), CONSTANT("AUTO_ROTATION_D"),
        CONSTANT("SWERVE_MAX_ANGULAR_VELOCITY"), CONSTANT("MAX_ROTATIONAL_ACCEL"));

    m_TotalTime = m_Trajectory.TotalTime().value();

    std::cout << "Completed constructing swerve command for " << trajectoryName << std::endl;
}

SwerveTrajectoryCommand::~SwerveTrajectoryCommand()
{
    delete m_Timer;
    delete m_HolonomicController;
}

bool SwerveTrajectoryCommand::IsComplete()
{
    // I think these units are correct but could be problem spot

    // More probably has to be added here

    return m_Timer->HasElapsed(m_TotalTime);
}

void SwerveTrajectoryCommand::Start(CowRobot* robot)
{
    m_Timer->Reset();
    m_Timer->Start();
}

void SwerveTrajectoryCommand::Handle(CowRobot* robot)
{
    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();

    frc::Trajectory::State targetPose = m_Trajectory.Sample(units::second_t { m_Timer->Get() });

    CowLib::CowChassisSpeeds chassisSpeeds = m_HolonomicController->Calculate(currentPose, targetPose, m_TargetAngle);

    // Vision align logic has to go here

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void SwerveTrajectoryCommand::Finish(CowRobot* robot)
{
    if (m_Stop) {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, false);
    }

    m_Timer->Stop();

    // Do we have to delete stuff here? Worth memory savings? idk
}
