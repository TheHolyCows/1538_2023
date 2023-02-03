#ifndef __COWLIB_COWSWERVEKINEMATICS_H__
#define __COWLIB_COWSWERVEKINEMATICS_H__

#include "./CowChassisSpeeds.h"
#include "./CowSwerveModuleState.h"

#include <algorithm>
#include <array>
#include <cmath>
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
        frc::SwerveDriveKinematics<4> *m_Kinematics;

        std::array<frc::Translation2d, 4> m_ModulePositions{};

    public:
        CowSwerveKinematics(double wheelBase);
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