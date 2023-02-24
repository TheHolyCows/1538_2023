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

    UpdatePipeline(limelightName, 0);

    if (GetPipeline(limelightName) != 0)
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

    UpdatePipeline(limelightName, 0);

    if (GetPipeline(limelightName) != 0)
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

    UpdatePipeline(limelightName, 0);

    if (GetPipeline(limelightName) != 0)
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

    UpdatePipeline(limelightName, 0);

    if (GetPipeline(limelightName) != 0)
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
    std::string positions[2] = { "front", "rear" };

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
