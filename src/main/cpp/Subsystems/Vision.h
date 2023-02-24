#ifndef __VISION_H__
#define __VISION_H__

#include "../CowConstants.h"
#include "../CowLib/CowLogger.h"
#include "../CowLib/CowPID.h"
#include "../CowLib/Utility.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <memory>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <optional>
#include <pathplanner/lib/PathPlanner.h>
#include <units/length.h>
#include <vector>

#define LL_PIPELINE_APRIL_TAG 0
#define LL_PIPELINE_REFLECTIVE_TAPE 1
#define LL_NAME_FRONT "limelight-front"
#define LL_NAME_BACK "limelight-back"

class Vision
{
private:
    static Vision *s_Instance;

    Vision();

    frc2::PIDController m_CubeYPID;
    frc2::PIDController m_CubeYawPID;
    frc2::PIDController m_ConeYPID;

    void UpdatePipeline(std::string limelightName, int id);
    int GetPipeline(std::string limelightName);

    std::string DetermineCorrectPosition();

public:
    static Vision *GetInstance();

    ~Vision() = default;

    /**
     * @brief Resets PID constants and controllers
     */
    void Reset();

    double CubeYPID();
    double CubeYawPID();

    bool CubeYAligned();
    bool CubeYawAligned();

    double ConeYPID();

    bool ConeYAligned();

    struct BotPoseResult
    {
        frc::Pose2d pose;
        double timestamp;
    };

    std::optional<BotPoseResult> GetBotPose();
};

#endif /* __VISION_H__ */