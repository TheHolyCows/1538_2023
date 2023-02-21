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
#include <pathplanner/lib/PathPlanner.h>
#include <units/length.h>
#include <vector>

class Vision
{
public:
    enum GamePiece
    {
        CUBE,
        CONE
    };

private:
    static Vision *s_Instance;

    Vision();

    frc2::PIDController m_ScoringYPID;
    frc2::PIDController m_ScoringYawPID;

public:
    static Vision *GetInstance();

    ~Vision() = default;

    /**
     * @brief Resets PID constants and controllers
     */
    void Reset();

    double ScoringYPID(GamePiece type);
    double ScoringYawPID();

    bool ScoringYAligned(GamePiece type);
    bool ScoringYawAligned();

    pathplanner::PathPlannerTrajectory GenerateTrajectoryToNearestAprilTag();
};

#endif /* __VISION_H__ */