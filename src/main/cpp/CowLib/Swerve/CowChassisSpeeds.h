#ifndef __COWLIB_COWCHASSISSPEEDS_H__
#define __COWLIB_COWCHASSISSPEEDS_H__

#include <cmath>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

namespace CowLib
{

    struct CowChassisSpeeds
    {
        // In feet per second
        double vx;
        double vy;

        // In degrees per second (Angular Velocity)
        double omega;

        static CowChassisSpeeds FromFieldRelativeSpeeds(double vx, double vy, double omega, double robotAngle)
        {
            return FromWPI(frc::ChassisSpeeds::FromFieldRelativeSpeeds(units::feet_per_second_t{ vx },
                                                                       units::feet_per_second_t{ vy },
                                                                       units::degrees_per_second_t{ omega },
                                                                       units::degree_t{ robotAngle }));
        }

        static CowChassisSpeeds FromWPI(frc::ChassisSpeeds chassisSpeeds)
        {
            return CowChassisSpeeds{ chassisSpeeds.vx.convert<units::feet_per_second>().value(),
                                     chassisSpeeds.vy.convert<units::feet_per_second>().value(),
                                     chassisSpeeds.omega.convert<units::degrees_per_second>().value() };
        }

        frc::ChassisSpeeds ToWPI()
        {
            return frc::ChassisSpeeds{ units::feet_per_second_t(vx),
                                       units::feet_per_second_t(vy),
                                       units::degrees_per_second_t(omega) };
        }
    };
} // namespace CowLib

#endif /* __COWLIB_COWCHASSISSPEEDS_H__ */