//==================================================
// Copyright (C) 2022 Team 1538 / The Holy Cows
// CowHolonomicController.h
// author: scott-semtner
// created on: 2022-12-14
//==================================================

#ifndef __COWLIB_COW_HOLONOMIC_CONTROLLER_H__
#define __COWLIB_COW_HOLONOMIC_CONTROLLER_H__

#include "./Swerve/CowChassisSpeeds.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>

namespace CowLib
{

    class CowHolonomicController
    {
    private:
        frc2::PIDController *m_XPIDController;
        frc2::PIDController *m_YPIDController;

        frc::ProfiledPIDController<units::radian> *m_ThetaPIDController;

        frc::HolonomicDriveController *m_HolonomicController;

    public:
        CowHolonomicController(double driveP,
                               double driveI,
                               double driveD,
                               double rotationP,
                               double rotationI,
                               double rotationD,
                               double maxRotationalVelocity,
                               double maxRotationalAccel);
        ~CowHolonomicController();

        CowChassisSpeeds Calculate(const frc::Pose2d &pose, frc::Trajectory::State desiredState, double desiredAngle);
        CowChassisSpeeds Calculate(const frc::Pose2d &currentPose,
                                   const frc::Pose2d &desiredPose,
                                   double maxLinearSpeed,
                                   double desiredAngle);

        frc::HolonomicDriveController *GetInternalController();
    };

} // namespace CowLib

#endif /* __COWLIB_COW_HOLONOMIC_CONTROLLER_H__ */
