#ifndef __COWLIB_COW_SWERVE_ODOMETRY_H__
#define __COWLIB_COW_SWERVE_ODOMETRY_H__

#include "./CowSwerveKinematics.h"
#include "./CowSwerveModulePosition.h"
#include "./CowSwerveModuleState.h"
#include "frc/smartdashboard/Field2d.h"
#include "frc/smartdashboard/SmartDashboard.h"

#include <algorithm>
#include <array>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/length.h>

namespace CowLib
{
    class CowSwerveOdometry
    {
    private:
        frc::Field2d m_Field;

        frc::SwerveDrivePoseEstimator<4> *m_PoseEstimator;
        frc::Pose2d m_Pose;

        CowSwerveKinematics *m_Kinematics;

        frc::Pose2d CreateWPIPose(double x, double y, double rotation);
        std::array<frc::SwerveModulePosition, 4>
        CreateWPIModulePositions(std::array<CowSwerveModulePosition, 4> modulePositions);

    public:
        CowSwerveOdometry(CowSwerveKinematics *kinematics,
                          double gyroAngle,
                          double initialX,
                          double initialY,
                          double initialRotation);
        ~CowSwerveOdometry();

        void Reset(double newX,
                   double newY,
                   double newRotation,
                   double gyroAngle,
                   std::array<CowLib::CowSwerveModulePosition, 4> modPositions);
        void Reset(frc::Pose2d pose, double gyroAngle, std::array<CowLib::CowSwerveModulePosition, 4> modPositions);

        double GetX();
        double GetY();
        double GetRotation();

        frc::Pose2d GetWPIPose();

        void Update(double gyroAngle, std::array<CowSwerveModulePosition, 4> modulePositions);

        frc::SwerveDrivePoseEstimator<4> *GetInternalPoseEstimator();
    };

} // namespace CowLib

#endif /* __COWLIB_COW_SWERVE_ODOMETRY_H__ */