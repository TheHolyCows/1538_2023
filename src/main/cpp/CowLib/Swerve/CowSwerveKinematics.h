#ifndef __COWLIB_COWSWERVEKINEMATICS_H__
#define __COWLIB_COWSWERVEKINEMATICS_H__

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

    // Custom eventually, rn just wrapper for wpilib
    class CowSwerveKinematics
    {
    private:
        static constexpr int NUM_MODULES = 4;

        mutable frc::Matrixd<NUM_MODULES * 2, 3> m_InverseKinematics;
        Eigen::HouseholderQR<frc::Matrixd<NUM_MODULES * 2, 3>> m_ForwardKinematics;

        frc::Translation2d m_PreviousCenterOfRotation;

        std::array<frc::Translation2d, NUM_MODULES> m_ModulePositions;

        std::array<CowSwerveModuleState, NUM_MODULES> m_ModuleStates;

    public:
        explicit CowSwerveKinematics(double wheelBase);
        ~CowSwerveKinematics();

        static void DesaturateSpeeds(std::array<CowSwerveModuleState, 4> *states, double maxSpeed);

        std::array<CowSwerveModuleState, 4>
        CalculateModuleStates(CowChassisSpeeds &chassisSpeeds, double centerOfRotationX, double centerOfRotationY);

        CowChassisSpeeds CalculateChassisSpeeds(std::array<CowSwerveModuleState, 4> &moduleStates);

        std::array<frc::Translation2d, 4> GetModulePositions();

        frc::SwerveDriveKinematics<4> *GetInternalKinematics();
    };

} // namespace CowLib

#endif /* __COWLIB_COWSWERVEKINEMATICS_H__ */