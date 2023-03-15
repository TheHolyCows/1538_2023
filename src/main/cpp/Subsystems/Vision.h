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
#include "ArmState.h"

#define LL_PIPELINE_APRIL_TAG_2D 0
#define LL_PIPELINE_APRIL_TAG_3D 2
#define LL_PIPELINE_REFLECTIVE_TAPE 1

class Vision
{
private:
    static Vision *s_Instance;

    Vision();

    frc2::PIDController m_CubeYPID;
    frc2::PIDController m_CubeYawPID;
    frc2::PIDController m_ConeYPID;

    void RequestPipeline(int id);
    int GetPipeline();
    std::shared_ptr<nt::NetworkTable> GetLimelightTable() ;
    bool HasTarget();
    std::vector<double> GetTargetPoseRobotRelative();
    double GetTargetX();

    [[maybe_unused]] bool m_Flipped;

public:
    static Vision *GetInstance();

    ~Vision() = default;

    /**
     * @brief Resets PID constants and controllers
     */
    void Reset();

    [[maybe_unused]] double Cube3dYPID();
    [[maybe_unused]] double Cube3dYawPID();

    [[maybe_unused]] bool Cube3dYAligned();
    [[maybe_unused]] bool Cube3dYawAligned();

    double CubeYPID();
    bool CubeYAligned();

    double ConeYPID();

    bool ConeYAligned();

    void SetInverted(bool flipped);

    void SetCargo(ARM_CARGO cargo);
};

#endif /* __VISION_H__ */