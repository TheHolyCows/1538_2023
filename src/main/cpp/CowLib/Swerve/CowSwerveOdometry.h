#ifndef __COWLIB_COWSWERVEODOMETRY_H__
#define __COWLIB_COWSWERVEODOMETRY_H__

#include "./CowSwerveKinematics.h"
#include "./CowSwerveModulePosition.h"

#include <algorithm>
#include <array>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/angle.h>
#include <units/length.h>

namespace CowLib
{

    class CowSwerveOdometry
    {
    private:
        frc::SwerveDriveOdometry<4> *m_Odometry;
        frc::Pose2d m_Pose;

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

        void Reset(double newX, double newY, double newRotation, double gyroAngle);

        double GetX();
        double GetY();
        double GetRotation();

        frc::Pose2d GetWPIPose();

        void Update(double gyroAngle, std::array<CowSwerveModulePosition, 4> modulePositions);

        frc::SwerveDriveOdometry<4> *GetInternalOdometry();
    };

} // namespace CowLib

#endif /* __COWLIB_COWSWERVEODOMETRY_H__ */