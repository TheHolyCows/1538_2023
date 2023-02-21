#include "Vision.h"

Vision *Vision::s_Instance = nullptr;

Vision *Vision::GetInstance()
{
    if (s_Instance == nullptr)
    {
        s_Instance = new Vision();
    }

    return s_Instance;
}

Vision::Vision()
    : m_ScoringYPID(CONSTANT("SCORING_Y_P"), CONSTANT("SCORING_Y_I"), CONSTANT("SCORING_Y_D")),
      m_ScoringYawPID(CONSTANT("SCORING_YAW_P"), CONSTANT("SCORING_YAW_I"), CONSTANT("SCORING_YAW_D"))
{
}

void Vision::Reset()
{
    m_ScoringYPID.SetPID(CONSTANT("SCORING_Y_P"), CONSTANT("SCORING_Y_I"), CONSTANT("SCORING_Y_D"));
    m_ScoringYawPID.SetPID(CONSTANT("SCORING_YAW_P"), CONSTANT("SCORING_YAW_I"), CONSTANT("SCORING_YAW_D"));

    m_ScoringYPID.Reset();
    m_ScoringYawPID.Reset();

    // TODO: find out why this was disabled. may have crashed
    // m_ScoringYPID->UpdateConstants(CONSTANT("SCORING_Y_P"), CONSTANT("SCORING_Y_I"), CONSTANT("SCORING_Y_D"), 0);
    // m_ScoringYawPID->UpdateConstants(CONSTANT("SCORING_YAW_P"),
    //                                  CONSTANT("SCORING_YAW_I"),
    //                                  CONSTANT("SCORING_YAW_D"),
    //                                  0);

    // m_ScoringYPID->Reset();
    // m_ScoringYawPID->Reset();
}

double Vision::ScoringYPID(GamePiece type)
{
    // bool targetFound = LimelightHelpers::GetTV();
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable("limelight")
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    // auto targetPoseVec = LimelightHelpers::GetTargetPose_RobotSpace();

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());
        return 0;
    }

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "vec length: %d", targetPoseVec.size());
    double targetX = targetPoseVec[0];

    if (type == GamePiece::CONE)
    {
        if (targetX > 0)
        {
            targetX -= 0.47;
        }
        else
        {
            targetX += 0.47;
        }
    }

    double yOutput = m_ScoringYPID.Calculate(targetX, 0.0);
    frc::SmartDashboard::PutNumber("april tag/y/target x", targetX);
    frc::SmartDashboard::PutNumber("april tag/y/y output", yOutput);

    if (fabs(targetX) < 0.01)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f within tolorance", targetX);

        return 0;
    }

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetX, yOutput);
    return yOutput;
}

double Vision::ScoringYawPID()
{
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable("limelight")
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());
        return 0;
    }

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "vec length: %d", targetPoseVec.size());
    double targetYaw = targetPoseVec[4];

    double yawOutput = m_ScoringYawPID.Calculate(targetYaw, 0.0);

    frc::SmartDashboard::PutNumber("april tag/yaw/target yaw", targetYaw);
    frc::SmartDashboard::PutNumber("april tag/yaw/yaw output", yawOutput);

    if (fabs(targetYaw) < 0.1)
    {
        yawOutput = 0;
    }

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetYaw, yawOutput);

    return yawOutput;
}

bool Vision::ScoringYAligned(GamePiece type)
{
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return false;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable("limelight")
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());

        return false;
    }

    double targetX = targetPoseVec[0];

    return fabs(targetX) < CONSTANT("SCORING_Y_TOLERANCE");
}

bool Vision::ScoringYawAligned()
{
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return false;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable("limelight")
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());

        return false;
    }

    double targetYaw = targetPoseVec[4];

    return fabs(targetYaw) < CONSTANT("SCORING_YAW_TOLERANCE");
}

pathplanner::PathPlannerTrajectory Vision::GenerateTrajectoryToNearestAprilTag()
{
    bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return pathplanner::PathPlannerTrajectory();
    }

    int tagID = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tid", -1);

    static frc::AprilTagFieldLayout aprilTagFieldLayout = frc::AprilTagFieldLayout();

    auto tagPose = aprilTagFieldLayout.GetTagPose(tagID);
    if (!tagPose.has_value())
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no tag pose");
        return pathplanner::PathPlannerTrajectory();
    }

    auto poseVec = nt::NetworkTableInstance::GetDefault()
                       .GetTable("limelight")
                       ->GetNumberArray("botpose", std::vector<double>(6));

    if (poseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  poseVec.size());

        return pathplanner::PathPlannerTrajectory();
    }

    // TODO: confirm these are the correct indexes
    frc::Pose2d pose
        = frc::Pose2d(units::meter_t{ poseVec[0] }, units::meter_t{ poseVec[1] }, units::degree_t{ poseVec[5] });

    pathplanner::PathPlannerTrajectory traj = pathplanner::PathPlanner::generatePath(
        pathplanner::PathConstraints{ 2_mps, 1_mps_sq },
        pathplanner::PathPoint(pose.Translation(), pose.Rotation()),
        pathplanner::PathPoint((*tagPose).Translation().ToTranslation2d(), (*tagPose).Rotation().ToRotation2d()));

    return traj;
}