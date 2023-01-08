#ifndef __COWLIB_COWSWERVEMODULESTATE_H__
#define __COWLIB_COWSWERVEMODULESTATE_H__

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <units/velocity.h>

namespace CowLib {

struct CowSwerveModuleState {
    double velocity;
    double angle;

    static CowSwerveModuleState FromWPI(frc::SwerveModuleState state)
    {
        return CowSwerveModuleState{state.speed.convert<units::feet_per_second>().value(), state.angle.Degrees().value()};
    }

    frc::SwerveModuleState ToWPI()
    {
        return frc::SwerveModuleState {
            units::feet_per_second_t(velocity),
            frc::Rotation2d { units::degree_t(angle) }
        };
    }
};

}

#endif /* __COWLIB_COWSWERVEMODULESTATE_H__ */