#ifndef __COWLIB_COWSWERVEODOMETRY_H__
#define __COWLIB_COWSWERVEODOMETRY_H__

#include <algorithm>
#include <array>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <units/angle.h>
#include <units/length.h>

#include "./CowSwerveKinematics.h"
#include "./CowSwerveModuleState.h"

namespace CowLib {

class CowSwerveOdometry {
private:
    frc::SwerveDriveOdometry<4>* m_Odometry;
    frc::Pose2d m_Pose;

    frc::Pose2d CreateWPIPose(double x, double y, double rotation);
    std::array<frc::SwerveModuleState, 4> CreateWPIModuleStates(std::array<CowSwerveModuleState, 4> moduleStates);

public:
    CowSwerveOdometry(CowSwerveKinematics* kinematics, double gyroAngle, double initialX, double initialY, double initialRotation);
    ~CowSwerveOdometry();

    void Reset(double newX, double newY, double newRotation, double gyroAngle);

    double GetX();
    double GetY();
    double GetRotation();

    frc::Pose2d GetWPIPose();

    void Update(double gyroAngle, std::array<CowSwerveModuleState, 4> moduleStates);
    void UpdateWithTime(double currentTime, double gyroAngle, std::array<CowSwerveModuleState, 4> moduleStates);

    frc::SwerveDriveOdometry<4>* GetInternalOdometry();
};

}

#endif /* __COWLIB_COWSWERVEODOMETRY_H__ */