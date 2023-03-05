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
      m_ConeYPID(CONSTANT("CONE_Y_P"), CONSTANT("CONE_Y_I"), CONSTANT("CONE_Y_D")),
      m_Flipped(false)
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
    if (GetPipeline() != LL_PIPELINE_APRIL_TAG)
    {
        return 0;
    }

    if (!HasTarget())
    {
        return 0;
    }

    auto targetPoseVec = GetTargetPoseRobotRelative();

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());
        return 0;
    }

    double targetX = targetPoseVec[0];

    double yOutput = m_CubeYPID.Calculate(targetX, 0.0);
    //    frc::SmartDashboard::PutNumber("april tag/y/target x", targetX);
    //    frc::SmartDashboard::PutNumber("april tag/y/y output", yOutput);

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetX, yOutput);

    return yOutput;
}

double Vision::CubeYawPID()
{
    if (GetPipeline() != LL_PIPELINE_APRIL_TAG)
    {
        return 0;
    }

    if (!HasTarget())
    {
        return 0;
    }

    auto targetPoseVec = GetTargetPoseRobotRelative();

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

    //        frc::SmartDashboard::PutNumber("april tag/yaw/target yaw", targetYaw);
    //        frc::SmartDashboard::PutNumber("april tag/yaw/yaw output", yawOutput);

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetYaw, yawOutput);

    return yawOutput;
}

bool Vision::CubeYAligned()
{
    if (GetPipeline() != LL_PIPELINE_APRIL_TAG)
    {
        return false;
    }

    if (!HasTarget())
    {
        return false;
    }

    auto targetPoseVec = GetTargetPoseRobotRelative();

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
    if (GetPipeline() != LL_PIPELINE_APRIL_TAG)
    {
        return false;
    }

    if (!HasTarget())
    {
        return false;
    }

    auto targetPoseVec = GetTargetPoseRobotRelative();

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
    if (GetPipeline() != LL_PIPELINE_REFLECTIVE_TAPE)
    {
        return 0;
    }

    if (!HasTarget())
    {
        return 0;
    }

    double tx = GetTargetX();

    double output = m_ConeYPID.Calculate(0, tx);

    return output;
}

bool Vision::ConeYAligned()
{
    if (GetPipeline() != LL_PIPELINE_REFLECTIVE_TAPE)
    {
        return false;
    }

    if (!HasTarget())
    {
        return false;
    }

    double tx = GetTargetX();

    return fabs(tx) < CONSTANT("CONE_Y_TOLERANCE");
}

// Only updates if it is different
void Vision::RequestPipeline(int id)
{
    if (GetPipeline() != id)
    {
        GetLimelightTable()->PutNumber("pipeline", id);
    }
}

int Vision::GetPipeline()
{
    return (int) GetLimelightTable()->GetNumber("pipeline", -1);
}

std::optional<Vision::BotPoseResult> Vision::GetBotPose()
{
    //    std::string limelightName = "";
    //    //    std::string limelightName = DetermineCorrectPosition();
    //
    //    if (limelightName == "none")
    //    {
    //        return std::nullopt;
    //    }
    //
    //    RequestPipeline(, LL_PIPELINE_APRIL_TAG);
    //
    //    if (GetPipeline() != LL_PIPELINE_APRIL_TAG)
    //    {
    //        return std::nullopt;
    //    }
    //
    //    auto botPoseVec = nt::NetworkTableInstance::GetDefault()
    //                          .GetTable(limelightName)
    //                          ->GetNumberArray("botpose", std::vector<double>(7));
    //
    //    if (botPoseVec.size() != 7)
    //    {
    //        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
    //                                  "bot pose array has incorrect length %f",
    //                                  botPoseVec.size());
    //
    //        return std::nullopt;
    //    }
    //
    //    // TODO: match up these indices with the what they actually do. This is a guess rn
    //    frc::Translation2d translation
    //        = frc::Translation2d(units::meter_t{ botPoseVec[0] }, units::meter_t{ botPoseVec[1] });
    //    frc::Rotation2d rotation = frc::Rotation2d(units::degree_t{ botPoseVec[5] });
    //
    //    frc::Pose2d pose = frc::Pose2d(translation, rotation);
    //
    //    double timestamp = frc::Timer::GetFPGATimestamp().value() - (botPoseVec[6] / 1000.0);
    //
    //    return BotPoseResult{ pose, timestamp };

    return std::nullopt;
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

void Vision::SetFlipped(bool flipped)
{
    m_Flipped = flipped;
}

std::shared_ptr<nt::NetworkTable> Vision::GetLimelightTable() const
{
    std::string name = (m_Flipped ? "limelight-front" : "limelight-back");

    return nt::NetworkTableInstance::GetDefault().GetTable(name);
    //    m_LimelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

bool Vision::HasTarget()
{
    return GetLimelightTable()->GetNumber("tv", 0.0) == 1.0;
}

std::vector<double> Vision::GetTargetPoseRobotRelative()
{
    return GetLimelightTable()->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
}

double Vision::GetTargetX()
{
    return GetLimelightTable()->GetNumber("tx", 0.0);
}

void Vision::SetCargo(ARM_CARGO cargo)
{
    switch (cargo)
    {
    case ARM_CARGO::CG_CUBE :
        RequestPipeline(LL_PIPELINE_APRIL_TAG);
        break;
    case ARM_CARGO::CG_CONE :
        RequestPipeline(LL_PIPELINE_REFLECTIVE_TAPE);
        break;
    default :
        break;
    }
}