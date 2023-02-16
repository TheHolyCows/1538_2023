#ifndef __COWLIB_COWSWERVEMODULESTATE_H__
#define __COWLIB_COWSWERVEMODULESTATE_H__

#include "ExtendedWPISwerveModuleState.h"

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/velocity.h>

namespace CowLib
{

    struct CowSwerveModuleState
    {
        double velocity;
        double angle;

        // degrees per second
        double omega = 0;

        static CowSwerveModuleState FromWPI(frc::SwerveModuleState state)
        {
            return CowSwerveModuleState{ state.speed.convert<units::feet_per_second>().value(),
                                         state.angle.Degrees().value() };
        }

        frc::SwerveModuleState ToWPI() const
        {
            return frc::SwerveModuleState{ units::feet_per_second_t(velocity),
                                           frc::Rotation2d{ units::degree_t(angle) } };
        }

        static CowSwerveModuleState FromWPIExtended(ExtendedWPISwerveModuleState state)
        {
            return CowSwerveModuleState{ state.speed.convert<units::feet_per_second>().value(),
                                         state.angle.Degrees().value(),
                                         state.omega.convert<units::degrees_per_second>().value() };
        }

        ExtendedWPISwerveModuleState ToWPIExtended() const
        {
            return ExtendedWPISwerveModuleState{ units::feet_per_second_t(velocity),
                                                 frc::Rotation2d{ units::degree_t(angle) },
                                                 units::degrees_per_second_t(omega) };
        }
    };

} // namespace CowLib

#endif /* __COWLIB_COWSWERVEMODULESTATE_H__ */