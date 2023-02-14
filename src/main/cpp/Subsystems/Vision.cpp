#include "Vision.h"

#include "Limelight.h"

Vision &Vision::GetInstance()
{
    static Vision instance;

    return instance;
}

Vision::Vision()
{
    m_ScoringYPID = std::make_unique<CowLib::CowPID>(CONSTANT("SCORING_Y_P"),
                                                     CONSTANT("SCORING_Y_I"),
                                                     CONSTANT("SCORING_Y_D"),
                                                     0);
    m_ScoringYPID = std::make_unique<CowLib::CowPID>(CONSTANT("SCORING_YAW_P"),
                                                     CONSTANT("SCORING_YAW_I"),
                                                     CONSTANT("SCORING_YAW_D"),
                                                     0);
}

void Vision::Reset()
{
    m_ScoringYPID->UpdateConstants(CONSTANT("SCORING_Y_P"), CONSTANT("SCORING_Y_I"), CONSTANT("SCORING_Y_D"), 0);
    m_ScoringYawPID->UpdateConstants(CONSTANT("SCORING_YAW_P"),
                                     CONSTANT("SCORING_YAW_I"),
                                     CONSTANT("SCORING_YAW_D"),
                                     0);

    m_ScoringYPID->Reset();
    m_ScoringYawPID->Reset();
}

double Vision::ScoringYPID(GamePiece type)
{
    bool targetFound = LimelightHelpers::GetTV();
    // bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    // auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
    //                          .GetTable("limelight")
    //                          ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    auto targetPoseVec = LimelightHelpers::GetTargetPose_RobotSpace();

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());
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

    m_ScoringYPID->SetSetpoint(targetX);
    double yOutput = m_ScoringYPID->Calculate(0.0);
    if (fabs(targetX) < 0.1)
    {
        yOutput = 0;
    }

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetX, yOutput);
    return yOutput;
}

double Vision::ScoringYawPID()
{
    bool targetFound = LimelightHelpers::GetTV();
    // bool targetFound = nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("tv", 0) == 1;
    if (!targetFound)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "no target found");
        return 0;
    }

    // auto targetPoseVec = nt::NetworkTableInstance::GetDefault()
    //                          .GetTable("limelight")
    //                          ->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
    auto targetPoseVec = LimelightHelpers::GetTargetPose_RobotSpace();

    if (targetPoseVec.size() != 6)
    {
        CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_ERR,
                                  "April tag targeting array has incorrect length %f",
                                  targetPoseVec.size());
    }

    // CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "vec length: %d", targetPoseVec.size());
    double targetYaw = targetPoseVec[4];

    m_ScoringYPID->SetSetpoint(targetYaw);
    double yawOutput = m_ScoringYPID->Calculate(0.0);
    if (fabs(targetYaw) < 0.1)
    {
        yawOutput = 0;
    }

    CowLib::CowLogger::LogMsg(CowLib::CowLogger::LOG_DBG, "april tag y target: %f output: %f", targetYaw, yawOutput);

    return yawOutput;
}