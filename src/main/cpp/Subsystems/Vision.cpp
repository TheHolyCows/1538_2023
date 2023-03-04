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
    : m_CubeYPID(CONSTANT("CUBE_Y_P"), CONSTANT("CUBE_Y_I"), CONSTANT("CUBE_Y_D")),
      m_CubeYawPID(CONSTANT("CUBE_YAW_P"), CONSTANT("CUBE_YAW_I"), CONSTANT("CUBE_YAW_D")),
      m_ConeYPID(CONSTANT("CONE_Y_P"), CONSTANT("CONE_Y_I"), CONSTANT("CONE_Y_D"))
{
}

void Vision::Reset()
{
    m_CubeYPID.SetPID(CONSTANT("CUBE_Y_P"), CONSTANT("CUBE_Y_I"), CONSTANT("CUBE_Y_D"));
    m_CubeYawPID.SetPID(CONSTANT("CUBE_YAW_P"), CONSTANT("CUBE_YAW_I"), CONSTANT("CUBE_YAW_D"));
    m_ConeYPID.SetPID(CONSTANT("CONE_Y_P"), CONSTANT("CONE_Y_I"), CONSTANT("CONE_Y_D"));

    m_CubeYPID.Reset();
    m_CubeYawPID.Reset();
    m_ConeYPID.Reset();
}

double Vision::CubeYPID()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_APRIL_TAG);

    if (GetPipeline(limelightName) != LL_PIPELINE_APRIL_TAG)
    {
        return 0;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable(limelightName)
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

    double yOutput = m_CubeYPID.Calculate(targetX, 0.0);
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

double Vision::CubeYawPID()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_APRIL_TAG);

    if (GetPipeline(limelightName) != LL_PIPELINE_APRIL_TAG)
    {
        return 0;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable(limelightName)
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

    double yawOutput = m_CubeYawPID.Calculate(targetYaw, 0.0);

    frc::SmartDashboard::PutNumber("april tag/yaw/target yaw", targetYaw);
    frc::SmartDashboard::PutNumber("april tag/yaw/yaw output", yawOutput);

    if (fabs(targetYaw) < 0.1)
    {
        yawOutput = 0;
    }

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetYaw, yawOutput);

    return yawOutput;
}

bool Vision::CubeYAligned()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return false;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_APRIL_TAG);

    if (GetPipeline(limelightName) != LL_PIPELINE_APRIL_TAG)
    {
        return false;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable(limelightName)
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());

        return false;
    }

    double targetX = targetPoseVec[0];

    return fabs(targetX) < CONSTANT("CUBE_Y_TOLERANCE");
}

bool Vision::CubeYawAligned()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return false;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_APRIL_TAG);

    if (GetPipeline(limelightName) != LL_PIPELINE_APRIL_TAG)
    {
        return false;
    }

    auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
                             .GetTable(limelightName)
                             ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());

        return false;
    }

    double targetYaw = targetPoseVec[4];

    return fabs(targetYaw) < CONSTANT("CUBE_YAW_TOLERANCE");
}

double Vision::ConeYPID()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_REFLECTIVE_TAPE);

    if (GetPipeline(limelightName) != LL_PIPELINE_REFLECTIVE_TAPE)
    {
        return 0;
    }

    // get tx
    double tx     = nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->GetNumber("tx", 0.0);
    double output = m_ConeYPID.Calculate(0, tx);

    return output;
}

bool Vision::ConeYAligned()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_REFLECTIVE_TAPE);

    if (GetPipeline(limelightName) != LL_PIPELINE_REFLECTIVE_TAPE)
    {
        return 0;
    }

    // get tx
    double tx = nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->GetNumber("tx", 0.0);

    return fabs(tx) < CONSTANT("CONE_Y_TOLERANCE");
}

// Only updates if it is different
void Vision::UpdatePipeline(std::string limelightName, int id)
{
    if (GetPipeline(limelightName) != id)
    {
        nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->PutNumber("pipeline", id);
    }
}

int Vision::GetPipeline(std::string limelightName)
{
    return nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->GetNumber("pipeline", -1);
}

std::string Vision::DetermineCorrectPosition()
{
    std::string positions[2] = { LL_NAME_FRONT, LL_NAME_BACK };

    for (auto position : positions)
    {
        // get tv for each limelight
        int tv = nt::NetworkTableInstance::GetDefault().GetTable(position)->GetNumber("tv", 0);

        if (tv == 1)
        {
            return position;
        }
    }

    return "none";
}

std::optional<Vision::BotPoseResult> Vision::GetBotPose()
{
    std::string limelightName = DetermineCorrectPosition();

    if (limelightName == "none")
    {
        return std::nullopt;
    }

    UpdatePipeline(limelightName, LL_PIPELINE_APRIL_TAG);

    if (GetPipeline(limelightName) != LL_PIPELINE_APRIL_TAG)
    {
        return std::nullopt;
    }

    auto botPoseVec = nt::NetworkTableInstance::GetDefault()
                          .GetTable(limelightName)
                          ->GetNumberArray("botpose", std::vector<double>(7));

    if (botPoseVec.size() != 7)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "bot pose array has incorrect length %f",
                                  botPoseVec.size());

        return std::nullopt;
    }

    // TODO: match up these indices with the what they actually do. This is a guess rn
    frc::Translation2d translation
        = frc::Translation2d(units::meter_t{ botPoseVec[0] }, units::meter_t{ botPoseVec[1] });
    frc::Rotation2d rotation = frc::Rotation2d(units::degree_t{ botPoseVec[5] });

    frc::Pose2d pose = frc::Pose2d(translation, rotation);

    double timestamp = frc::Timer::GetFPGATimestamp().value() - (botPoseVec[6] / 1000.0);

    return BotPoseResult{ pose, timestamp };
}

std::optional<pathplanner::PathPlannerTrajectory> Vision::GenerateTrajectoryToCube()
{
//    // Get current botpose
//    // TODO: make this from odometry
//    auto botPoseResult = GetBotPose();
//    if (!botPoseResult.has_value())
//    {
//        return std::nullopt;
//    }
//
//    frc::Pose2d pose = (*botPoseResult).pose;
//
//    // Get tag id
//    // Botpose will already fail if incorrect pipeline so no need to check
//    std::string limelightName = DetermineCorrectPosition();
//
//    if (limelightName == "none")
//    {
//        return std::nullopt;
//    }
//
//    int tagId = nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->GetNumber("tid", -1);
//
//    if (tagId == -1)
//    {
//        return std::nullopt;
//    }
//
//    // use wpilib to get april tag location
//    static frc::AprilTagFieldLayout fieldLayout = frc::AprilTagFieldLayout();
//    auto tagLocationResult = fieldLayout.GetTagPose(tagId);
//    if (!tagLocationResult.has_value())
//    {
//        return std::nullopt;
//    }
//
//    frc::Pose2d tagLocation = (*tagLocationResult).ToPose2d();
//
//    frc::Translation2d targetTranslation = frc::Translation2d(pose.X(), tagLocation.Y());
//
//    frc::Rotation2d targetRotation = frc::Rotation2d();
//
//
//
//    // Make path from current position to april tag (with same x as current)

    return std::nullopt;
}
