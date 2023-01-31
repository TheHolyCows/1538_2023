#ifndef __SWERVE_DRIVE_H__
#define __SWERVE_DRIVE_H__

#include "../CowConstants.h"
#include "../CowLib/CowMotorController.h"
#include "../CowLib/CowPID.h"
#include "../CowLib/Swerve/CowSwerveKinematics.h"
#include "../CowLib/Swerve/CowSwerveModulePosition.h"
#include "../CowLib/Swerve/CowSwerveOdometry.h"
#include "../CowLib/Utility.h"
#include "../CowPigeon.h"
#include "SwerveModule.h"

#include <algorithm>
#include <array>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <memory>

class SwerveDrive
{
private:
    std::array<SwerveModule *, 4> m_Modules{};

    frc::Rotation2d m_Angle = frc::Rotation2d{ 0_deg };

    frc::Pose2d m_Pose{ 0_m, 0_m, 0_deg };

    CowPigeon *m_Gyro;

    CowLib::CowSwerveKinematics *m_Kinematics;
    CowLib::CowSwerveOdometry *m_Odometry;

    bool m_Locked;

    CowLib::CowChassisSpeeds m_PrevChassisSpeeds{ 0, 0, 0 };

    double m_PreviousRotationError = 0;
    double m_PreviousXError        = 0;
    double m_PreviousYError        = 0;

    frc::Field2d m_Field;

    // CowLib::CowPID* m_VisionPIDController;

    // frc::ChassisSpeeds m_speeds;

public:
    struct ModuleConstants
    {
        int driveMotorId;
        int rotationMotorId;
        int encoderId;
        double encoderOffset;
    };

    SwerveDrive(ModuleConstants constants[4], double wheelBase);
    ~SwerveDrive();

    void SetVelocity(double x,
                     double y,
                     double rotation,
                     bool isFieldRelative     = true,
                     double centerOfRotationX = 0,
                     double centerOfRotationY = 0);

    void SetVelocity(CowLib::CowChassisSpeeds chassisSpeeds,
                     bool isFieldRelative     = true,
                     double centerOfRotationX = 0,
                     double centerOfRotationY = 0);

    // void SetVisionAlignVelocity(double x, double y, double rotation, bool isFieldRelative = true);

    frc::Pose2d GetPose() { return m_Odometry->GetWPIPose(); }

    double GetPoseX();
    double GetPoseY();
    double GetPoseRot();

    bool GetLocked() const;
    void SetLocked(bool isLocked);

    void ResetConstants();
    void ResetEncoders();

    void Reset()
    {
        ResetConstants();
        ResetEncoders();
        ResetOdometry(frc::Pose2d(0_ft, 0_ft, 0_deg));
    }

    void ResetOdometry(frc::Pose2d pose = frc::Pose2d{ 0_m, 0_m, 0_deg });

    void Handle();
};

#endif /* __SWERVE_DRIVE_H__ */