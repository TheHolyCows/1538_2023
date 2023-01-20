#include "HoldPositionCommand.h"

HoldPositionCommand::HoldPositionCommand(frc::Pose2d pose,
                                         double time,
                                         double maxVelocity,
                                         bool stopAtEnd,
                                         bool resetOdometry)
{
    m_Timer         = new CowLib::CowTimer();
    m_Stop          = stopAtEnd;
    m_Pose          = pose;
    m_MaxVelocity   = maxVelocity;
    m_ResetOdometry = resetOdometry;

    m_XController        = new frc2::PIDController(CONSTANT("AUTO_HOLD_DRIVE_P"),
                                            CONSTANT("AUTO_HOLD_DRIVE_I"),
                                            CONSTANT("AUTO_HOLD_DRIVE_D"));
    m_YController        = new frc2::PIDController(CONSTANT("AUTO_HOLD_DRIVE_P"),
                                            CONSTANT("AUTO_HOLD_DRIVE_I"),
                                            CONSTANT("AUTO_HOLD_DRIVE_D"));
    m_RotationController = new frc2::PIDController(CONSTANT("AUTO_HOLD_ROTATION_P"),
                                                   CONSTANT("AUTO_HOLD_ROTATION_I"),
                                                   CONSTANT("AUTO_HOLD_ROTATION_D"));

    // m_HolonomicController = new CowLib::CowHolonomicController(CONSTANT("AUTO_DRIVE_P"),
    //                                                            CONSTANT("AUTO_DRIVE_I"),
    //                                                            CONSTANT("AUTO_DRIVE_D"),
    //                                                            CONSTANT("AUTO_ROTATION_P"),
    //                                                            CONSTANT("AUTO_ROTATION_I"),
    //                                                            CONSTANT("AUTO_ROTATION_D"),
    //                                                            CONSTANT("SWERVE_MAX_ANGULAR_VELOCITY"),
    //                                                            CONSTANT("SWERVE_MAX_ROTATIONAL_ACCEL"));

    m_TotalTime = time;
}

HoldPositionCommand::~HoldPositionCommand()
{
    delete m_Timer;
    // delete m_HolonomicController;
    delete m_XController;
    delete m_YController;
    delete m_RotationController;
}

bool HoldPositionCommand::IsComplete()
{
    // I think these units are correct but could be problem spot

    // More probably has to be added here

    return m_Timer->HasElapsed(m_TotalTime);
}

void HoldPositionCommand::Start(CowRobot *robot)
{
    if (m_ResetOdometry)
    {
        robot->GetDrivetrain()->ResetOdometry(m_Pose);

        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                                  "Initial pose: x %f, y %f, d %f",
                                  m_Pose.X().convert<units::foot>().value(),
                                  m_Pose.Y().convert<units::foot>().value(),
                                  m_Pose.Rotation().Degrees().value());
    }

    m_Timer->Reset();
    m_Timer->Start();
}

void HoldPositionCommand::Handle(CowRobot *robot)
{
    frc::Pose2d currentPose = robot->GetDrivetrain()->GetPose();
    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                              "current: x %f y %f angle %f",
                              currentPose.X().value(),
                              currentPose.Y().value(),
                              currentPose.Rotation().Degrees().value());
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "set to:  x %f y %f angle %f",
    //                           m_Pose.X().value(),
    //                           m_Pose.Y().value(),
    //                           m_Pose.Rotation().Degrees().value());

    double vx = m_XController->Calculate(currentPose.X().convert<units::foot>().value(),
                                         m_Pose.X().convert<units::foot>().value());
    double vy = m_YController->Calculate(currentPose.Y().convert<units::foot>().value(),
                                         m_Pose.Y().convert<units::foot>().value());
    double omega
        = m_YController->Calculate(currentPose.Rotation().Degrees().value(), m_Pose.Rotation().Degrees().value());

    auto chassisSpeeds = CowLib::CowChassisSpeeds{ vx, vy, omega };

    // CowLib::CowChassisSpeeds chassisSpeeds
    //     = m_HolonomicController->Calculate(currentPose, m_Pose, m_MaxVelocity, m_Pose.Rotation().Degrees().value());
    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
    //                           "vx: %f, vy: %f, o: %f",
    //                           chassisSpeeds.vx,
    //                           chassisSpeeds.vy,
    //                           chassisSpeeds.omega);

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG,
                              "set to:  x %f y %f angle %f",
                              chassisSpeeds.vx,
                              chassisSpeeds.vy,
                              chassisSpeeds.omega);

    // Vision align logic has to go here

    robot->GetDrivetrain()->SetVelocity(chassisSpeeds, false);
}

void HoldPositionCommand::Finish(CowRobot *robot)
{
    if (m_Stop)
    {
        robot->GetDrivetrain()->SetVelocity(0, 0, 0, true);
    }

    m_Timer->Stop();
}
