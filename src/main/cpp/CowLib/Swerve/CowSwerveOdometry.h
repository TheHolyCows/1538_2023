#ifndef __COWLIB_COW_SWERVE_ODOMETRY_H__
#define __COWLIB_COW_SWERVE_ODOMETRY_H__

#include "../Geometry/Pose2d.h"
#include "./CowSwerveKinematics.h"
#include "./CowSwerveModulePosition.h"
#include "./CowSwerveModuleState.h"
#include "CowChassisSpeeds.h"

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
        CowSwerveKinematics *m_Kinematics;

        Pose2d m_Pose;

        CowChassisSpeeds m_ChassisSpeeds;

        double m_PreviousTime = -1;

        Rotation2d m_PreviousAngle;
        std::array<double, 4> m_PreviousDistances;

    public:
        CowSwerveOdometry(CowSwerveKinematics *kinematics, Pose2d initialPose, std::array<double, 4> previousDistances);

        CowSwerveOdometry(CowSwerveKinematics *kinematics, Pose2d initialPose);

        CowSwerveOdometry(CowSwerveKinematics *kinematics);

        ~CowSwerveOdometry();

        void Reset(Pose2d pose);
        void Reset(Pose2d pose, std::array<double, 4> previousDistances);

        Pose2d GetPose() const;

        frc::Pose2d GetWPIPose() { return frc::Pose2d(); }

        CowChassisSpeeds GetChassisSpeeds() const;

        Pose2d UpdateWithWheelConstraints(double currentTime,
                                          Rotation2d gyroAngle,
                                          std::array<CowSwerveModuleState, 4> moduleStates);

        void Update(double gyroAngle, std::array<CowSwerveModulePosition, 4> modulePositions);

        frc::SwerveDrivePoseEstimator<4> *GetInternalPoseEstimator();
    };

} // namespace CowLib

#endif /* __COWLIB_COW_SWERVE_ODOMETRY_H__ */