#ifndef __COWLIB_COWSWERVEKINEMATICS_H__
#define __COWLIB_COWSWERVEKINEMATICS_H__

#include "../Geometry/Rotation2d.h"
#include "../Geometry/Translation2d.h"
#include "CowChassisSpeeds.h"
#include "CowSwerveModuleState.h"
#include "Eigen/QR"

#include <algorithm>
#include <array>
#include <cmath>
#include <frc/EigenCore.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <iostream>

namespace CowLib
{
    class CowSwerveKinematics
    {
    private:
        static constexpr int NUM_MODULES = 4;

        mutable frc::Matrixd<NUM_MODULES * 2, 3> m_InverseKinematics;
        Eigen::HouseholderQR<frc::Matrixd<NUM_MODULES * 2, 3>> m_ForwardKinematics;

        Translation2d m_PreviousCenterOfRotation;

        std::array<Translation2d, NUM_MODULES> m_ModulePositions;

        mutable std::array<CowSwerveModuleState, NUM_MODULES> m_ModuleStates{};
        mutable std::array<Rotation2d, 4> m_ModuleRotations{};

    public:
        explicit CowSwerveKinematics(double wheelBase);
        ~CowSwerveKinematics();

        static void DesaturateSpeeds(std::array<CowSwerveModuleState, 4> *states, double maxSpeed);

        std::array<CowSwerveModuleState, 4> CalculateModuleStates(const CowChassisSpeeds &chassisSpeeds,
                                                                  const Translation2d centerOfRotation
                                                                  = Translation2d()) const;

        std::array<CowSwerveModuleState, 4>
        CalculateModuleStates(const CowChassisSpeeds &chassisSpeeds, const double corx, const double cory) const
        {
            return CalculateModuleStates(chassisSpeeds, Translation2d(corx, cory));
        }

        CowChassisSpeeds CalculateChassisSpeeds(const std::array<CowSwerveModuleState, 4> &moduleStates);

        CowChassisSpeeds CalculuateChassisSpeedsWithWheelConstraints(const std::array<CowSwerveModuleState, 4> &moduleStates);

        std::array<Translation2d, 4> GetModulePositions();

        frc::SwerveDriveKinematics<4> *GetInternalKinematics();
    };

} // namespace CowLib

#endif /* __COWLIB_COWSWERVEKINEMATICS_H__ */