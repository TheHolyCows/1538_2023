#ifndef __COWLIB_COWSWERVEKINEMATICS_H__
#define __COWLIB_COWSWERVEKINEMATICS_H__

#include <algorithm>
#include <array>
#include <cmath>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <iostream>

#include "./CowChassisSpeeds.h"
#include "./CowSwerveModuleState.h"

namespace CowLib {

// Custom eventually, rn just wrapper for wpilib
class CowSwerveKinematics {

private:
    frc::SwerveDriveKinematics<4>* m_Kinematics;

    std::array<frc::SwerveModuleState, 4> m_WPIModuleStates {};

public:
    CowSwerveKinematics(double wheelBase);
    ~CowSwerveKinematics();

    static void DesaturateSpeeds(std::array<CowSwerveModuleState, 4>* states, double maxSpeed);

    std::array<CowSwerveModuleState, 4> CalculateModuleStates(CowChassisSpeeds& chassisSpeeds);

    frc::SwerveDriveKinematics<4>* GetInternalKinematics();
};

}

#endif /* __COWLIB_COWSWERVEKINEMATICS_H__ */